using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;
using DFVolume;

public enum CollisionMode { PointSamplingActivation, ClosestPoint, None }
public enum OptimizationMode { GradientDescent, FrankWolfe }

public class ClothSimulatorModified : MonoBehaviour {
    [Header("Simulation Parameters")]
    public float timestep = 0.02f;
    public int iterationNum = 5;
    public float vertexMass = 2;

    [Header("External Forces")]
    public Vector3 gravity;

    [Header("Cloth Parameters")]
    public bool useExistingMesh;
    public int rows = 10;
    public int columns = 10;
    
    [Header("Distance Constraint")]
    public float distanceCompressionStiffness = 0.8f;
    public float distanceStretchStiffness = 0.8f;

    [Header("Bending Constraint")]
    public float bendingStiffness = 0.1f;
    public BendingMethod bendingMethod;

    [Header("Velocity Damping")]
    public DampingMethod dampingMethod;
    public float dampingStiffness = 0.02f;

    [Header("Point Constraints")]
    public PointConstraintType pointConstraintType;
    public int[] pointConstraintCustomIndices;

    // [TODO]: Setup multi-target
    [Header("Collision")]
    public CollisionMode collisionMode = CollisionMode.PointSamplingActivation;
    public GameObject target;

    // Simulation data
    private float nextFrameTime = 0f;
    private Vector3[] positions; 
    private Vector3[] projectedPositions;
    private Vector3[] velocities;
    private float[] frictions;
    private Triangle[] triangles;
    private float invMass;
    private List<Constraint> constraints = new List<Constraint>();
    private List<Constraint> collisionConstraints = new List<Constraint>();
    private List<PointConstraint> pointConstraints = new List<PointConstraint>();
    private int numParticles;

    // Unity data
    private Mesh mesh;
    private Mesh reverseMesh;
    
    // SDF data
    // [TODO]: Setup multi-target
    private Vector3[] x;
    private Vector3[] projectedX;
    private float[] distances;
    private Collider targetCollider;
    private Vector3 targetBounds;

    // Volume Data
    private VolumeData targetVolume;
    private int resolution;
    private float extent;
    
    CustomSampler mySampler;

    private void Start () {
        if (useExistingMesh) {
            // Create new mesh
            mesh = GetComponent<MeshFilter>().mesh;
        }
        else {
            mesh = Utility.CreateClothMesh(rows, columns);
            GetComponent<MeshFilter>().mesh = mesh;
        }
        mesh.MarkDynamic();

        numParticles = mesh.vertexCount;
        Vector3[] baseVertices = mesh.vertices;

        positions = new Vector3[numParticles];
        projectedPositions = new Vector3[numParticles];
        velocities = new Vector3[numParticles];
        frictions = new float[numParticles];

        // Create a new mesh for the opposite side
        CreateBackSide();

        // Initialize position, velocity and weight
        for (int i = 0; i < numParticles; i++) {
            positions[i] = baseVertices[i];
            projectedPositions[i] = positions[i];
            velocities[i] = Vector3.zero;
            frictions[i] = 1;
        }
        invMass = 1.0f / vertexMass;

        // Initialize triangles
        int[] triangleIndices = mesh.GetTriangles(0);
        triangles = new Triangle[triangleIndices.Length / 3];
        for (int i = 0; i < triangles.Length; i++) {
            triangles[i] = new Triangle(triangleIndices[i * 3], triangleIndices[i * 3 + 1], triangleIndices[i * 3 + 2]);
        }

        // SDF Start
        // Maintain x as world coordinates for simplicity
        x = new Vector3[triangles.Length];
        projectedX = new Vector3[triangles.Length];
        distances = new float[triangles.Length];

        // Modify positions to world coordinates before calculating constraint restlengths
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.TransformPoint(positions[i]);
        }

        // add constraints
        AddDistanceConstraints();
        AddPointConstraints();
        AddBendingConstraints();

        // modify positions to world coordinates before calculating constraint restlengths
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i]);
        }

        // Target information for meshes's SDF
        targetCollider = target.GetComponent<Collider>();
        targetBounds = target.GetComponent<Renderer>().bounds.size;

        if (targetCollider.GetType() == typeof(MeshCollider)) {
            targetVolume = target.GetComponent<Target>().volume;
            if (targetVolume) {
                resolution = targetVolume.resolution;
                extent = targetVolume.extent;
            }
        }

        mySampler = CustomSampler.Create("mySampler");
    }
    

    private void Update () {
        // Modify data to world coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.TransformPoint(positions[i]);
            projectedPositions[i] = transform.TransformPoint(projectedPositions[i]);
            velocities[i] = transform.TransformVector(velocities[i]);
        }
        
        // Calculate the timestep 
        nextFrameTime += Time.deltaTime;
        int iter = 0;
        while (nextFrameTime > 0) {
            if (nextFrameTime < timestep) {
                break;
            }

            float dt = Mathf.Min(nextFrameTime, timestep);
            nextFrameTime -= dt;
            iter++;

            // Step 1: apply external forces
            ApplyExternalForce(gravity, dt);

            // Step 2: damp velocity
            if (dampingMethod != DampingMethod.noDamping) {
                DampVelocity(dampingStiffness, dampingMethod);
            }

            // step 4: apply explicit Euler to positions based on velocity
            ApplyExplicitEuler(dt);
            // Or Verlet
            //ApplyVerlet(dt);

            // step 3: Frank - Wolfe
            calculateSDF();

            // step 5: clear current collisions and generate new collisions
            ClearCollisionConstraints();
            GenerateCollisionConstraints();

            // step 6-7: project constraints iterationNum times
            for (int j = 0; j < iterationNum; j++) {
                // satisfy all constraints
                SatisfyConstraints();
            }

            // satisfy pointConstraints
            SatisfyPointConstraints(dt);

            // step 8-9: apply projected positions to actual vertices
            UpdateVertices(dt);

            // step 10: update all velocities using friction
            ApplyFriction();
        }

        
        Vector3 newCenter = Vector3.zero;
        Vector3 delta = Vector3.zero;
        // recalculate the center of the mesh
        if (pointConstraintType == PointConstraintType.none && pointConstraintCustomIndices.Length == 0) {
            newCenter = GetComponentInChildren<Renderer>().bounds.center;
            delta = newCenter - transform.position;
        }
    

        // modify data to back to local coordinates
        for (int i = 0; i < numParticles; i++) {
            positions[i] = transform.InverseTransformPoint(positions[i] - delta);
            projectedPositions[i] = transform.InverseTransformPoint(projectedPositions[i] - delta);
            velocities[i] = transform.InverseTransformVector(velocities[i]);
        }

        if (pointConstraintType == PointConstraintType.none && pointConstraintCustomIndices.Length == 0) {
            transform.position = newCenter;
        }

        // update everything into Unity
        mesh.vertices = positions;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        reverseMesh.vertices = positions;
        Vector3[] reverseNormals = mesh.normals;
        for (int i = 0; i < reverseNormals.Length; i++) {
            reverseNormals[i] *= -1;
        }
        reverseMesh.normals = reverseNormals;

        //print(Time.deltaTime + ", " + iter);
    }


    private void calculateSDF() {
        // Check for proper setup
        if (!targetCollider) {
            return;
        }

        if (targetCollider.GetType() == typeof(MeshCollider) && targetVolume == null) {
            return;
        }

        // Calculate SDF values for each triangles
        for (int i = 0; i < triangles.Length; i++) {
            Vector3 a = projectedPositions[triangles[i].vertices[0]];
            Vector3 b = projectedPositions[triangles[i].vertices[1]];
            Vector3 c = projectedPositions[triangles[i].vertices[2]];

            projectedX[i] = (a + b + c) / 3;

            // Cull unnecessary calculations
            if (GetField(projectedX[i]) >= 0.0001f) {
                distances[i] = 0.0001f;
                continue;
            }

            // Search iterations
            for (int k = 0; k < 32; k++) {
                // Frank - wolfe
                Vector3 gradient = GetGradient(projectedX[i]);
                float da = Vector3.Dot(a, gradient);
                float db = Vector3.Dot(b, gradient);
                float dc = Vector3.Dot(c, gradient);
                    
                Vector3 s;

                if (da < db && da < dc) 
                    s = a;
                else if (db < da && db < dc)
                    s = b;
                else s = c;
                    
                float gamma = 2.0f /((float) i + 2.0f);
                    
                projectedX[i] = projectedX[i] + gamma * (projectedX[i] - s);
            }

            distances[i] = GetField(projectedX[i]);
        }
    }


    private void CreateBackSide() {
        GameObject newCloth = new GameObject("back");
        newCloth.transform.parent = transform;
        newCloth.transform.localPosition = Vector3.zero;
        newCloth.transform.localRotation = Quaternion.identity;
        newCloth.transform.localScale = new Vector3(1, 1, 1);
        newCloth.AddComponent<MeshRenderer>();
        newCloth.GetComponent<MeshRenderer>().material = GetComponent<MeshRenderer>().material;
        newCloth.AddComponent<MeshFilter>();
        reverseMesh = Utility.DeepCopyMesh(mesh);
        reverseMesh.MarkDynamic();

        // reverse the triangle order
        for (int m = 0; m < reverseMesh.subMeshCount; m++) {
            int[] triangles = reverseMesh.GetTriangles(m);
            for (int i = 0; i < triangles.Length; i += 3) {
                int temp = triangles[i + 0];
                triangles[i + 0] = triangles[i + 1];
                triangles[i + 1] = temp;
            }
            reverseMesh.SetTriangles(triangles, m);
        }
        newCloth.GetComponent<MeshFilter>().mesh = reverseMesh;
    }


    private void ApplyExternalForce(Vector3 gravity, float dt) {
        for (int i = 0; i < numParticles; i++) {
            velocities[i] += gravity * invMass * dt;
        }
    }


    private void DampVelocity(float dampingStiffness, DampingMethod method) {
        // dumb and simple way
        if (method == DampingMethod.simpleDamping) { 
            for (int i = 0; i < numParticles; i++) {
                velocities[i] *= 0.998f;
            }
        }
        else {
            // first compute the center of mass's position and velocity
            Vector3 centerMassPosition = Vector3.zero;
            Vector3 centerMassVelocity = Vector3.zero;
            float massSum = 0;
            float mass = invMass;

            for (int i = 0; i < numParticles; i++) {
                massSum += mass;
                centerMassPosition += positions[i] * mass;
                centerMassVelocity += velocities[i] * mass;
            }
            centerMassPosition /= massSum;
            centerMassVelocity /= massSum;

            // now compute L = sum of all r cross mass * velocity
            // also compute I = sum of rs * rs_transpose * mass
            Vector3 L = Vector3.zero;
            Matrix4x4 I = Matrix4x4.zero;

            for (int i = 0; i < numParticles; i++) {
                //  r is position - center of mass
                Vector3 r = positions[i] - centerMassPosition;
                L += Vector3.Cross(r, mass * velocities[i]);
                // rs = [ 0      -r.z     r.y  ]
                //        r.z     0      -r.x
                //       -r.y     r.x     0
                Matrix4x4 rs = Matrix4x4.zero;
                rs[0, 1] = -r[2];
                rs[0, 2] = r[1];
                rs[1, 0] = r[2];
                rs[1, 2] = -r[0];
                rs[2, 0] = -r[1];
                rs[2, 1] = r[0];
                rs[3, 3] = 1;
                Matrix4x4 temp = Utility.ScaleMatrixByFloat(rs * rs.transpose, mass);
                I = Utility.AddMatrices(I, temp);
            }

            // w = I_inverse * L
            I[3, 3] = 1;
            Matrix4x4 I_inv = I.inverse;
            Vector3 w = I_inv * L;

            // apply w back into velocities
            for (int i = 0; i < numParticles; i++) {
                Vector3 r = positions[i] - centerMassPosition;
                Vector3 dv = centerMassVelocity + Vector3.Cross(w, r) - velocities[i];
                velocities[i] += dampingStiffness * dv;
            }
        }
    }


    private void ClearCollisionConstraints() {
        collisionConstraints.Clear();
        for (int i = 0; i < numParticles; i++) {
            frictions[i] = 1;
        }
    }


    private void GenerateCollisionConstraints() {
        //int counter = 0;
        for (int i = 0; i < triangles.Length; i++) {
                if (!target.activeSelf) continue;

                if (distances[i] >= 0.0001f) {
                    //counter++;
                    continue;
                }

                int a = triangles[i].vertices[0];
                int b = triangles[i].vertices[1];
                int c = triangles[i].vertices[2];
                
                bool collided = false;

                if (targetCollider.GetType() == typeof(SphereCollider)) {
                    if (collisionMode == CollisionMode.PointSamplingActivation) {
                        Vector3 center = targetCollider.GetComponent<SphereCollider>().center + targetCollider.transform.position;
                        float radius = targetCollider.GetComponent<SphereCollider>().radius * targetCollider.transform.lossyScale.x;

                        if ((projectedPositions[a] - center).magnitude < radius) {
                            collided = true;
                            collisionConstraints.Add(new SphereCollisionConstraint(a, center, radius, positions[a], projectedPositions[a]));
                        }

                        if ((projectedPositions[b] - center).magnitude < radius) {
                            collided = true;
                            collisionConstraints.Add(new SphereCollisionConstraint(b, center, radius, positions[b], projectedPositions[b]));
                        }

                        if ((projectedPositions[c] - center).magnitude < radius) {
                            collided = true;
                            collisionConstraints.Add(new SphereCollisionConstraint(c, center, radius, positions[c], projectedPositions[c]));
                        }
                    }
                    else if (collisionMode == CollisionMode.ClosestPoint) {
                        Vector3 center = targetCollider.GetComponent<SphereCollider>().center + targetCollider.transform.position;
                        float radius = targetCollider.GetComponent<SphereCollider>().radius * targetCollider.transform.lossyScale.x;

                        if ((projectedX[i] - center).magnitude < radius) {
                            collided = true;
                            int[] cur = {a, b, c};
                            collisionConstraints.Add(new NewSphereCollisionConstraint(x[i],projectedX[i],cur, center, radius));
                        }
                    }
                }

                else if (targetCollider.GetType() == typeof(BoxCollider)) {
                    if (collisionMode == CollisionMode.PointSamplingActivation) {
                        Vector3 extent = 0.5f * targetCollider.GetComponent<BoxCollider>().size;

                        Vector3 localProjectedPosition = targetCollider.transform.InverseTransformPoint(projectedPositions[a]);
                        if (Utility.IsPointInCube(localProjectedPosition, extent)) {
                            collided = true;
                            Vector3 localPosition = targetCollider.transform.InverseTransformPoint(positions[a]);
                            collisionConstraints.Add(new CubeCollisionConstraint(a, localPosition, localProjectedPosition, extent, targetCollider.transform));
                        }

                        localProjectedPosition = targetCollider.transform.InverseTransformPoint(projectedPositions[b]);
                        if (Utility.IsPointInCube(localProjectedPosition, extent)) {
                            collided = true;
                            Vector3 localPosition = targetCollider.transform.InverseTransformPoint(positions[b]);
                            collisionConstraints.Add(new CubeCollisionConstraint(b, localPosition, localProjectedPosition, extent, targetCollider.transform));
                        }

                        localProjectedPosition = targetCollider.transform.InverseTransformPoint(projectedPositions[c]);
                        if (Utility.IsPointInCube(localProjectedPosition, extent)) {
                            collided = true;
                            Vector3 localPosition = targetCollider.transform.InverseTransformPoint(positions[c]);
                            collisionConstraints.Add(new CubeCollisionConstraint(c, localPosition, localProjectedPosition, extent, targetCollider.transform));
                        }
                    }
                }
                
                else if (targetCollider.GetType() == typeof(MeshCollider)) {
                    int[] triangleIndices = { triangles[i].vertices[0], triangles[i].vertices[1], triangles[i].vertices[2]};

                    Vector3 forward = (projectedX[i] - x[i]).normalized;
                    Vector3 o = x[i] - forward * 500.0f;

                    RaycastHit hitInfo;

                    Ray rayForward = new Ray(x[i], forward);
                    Ray rayBackward = new Ray(o, forward);

                    bool hitBackward = false, hitForward = false;
                    Vector3 pointBackward = Vector3.zero, pointForward = Vector3.zero;
                    Vector3 normalBackward = Vector3.zero, normalForward = Vector3.zero;
                    float distBackward = 0.0f, distForward = 0.0f;

                    if (Physics.Raycast(rayForward, out hitInfo, (projectedX[i] - x[i]).magnitude)) {
                        hitForward = true;
                        pointForward = hitInfo.point;
                        normalForward = hitInfo.normal;
                        distForward = hitInfo.distance;
                    }

                    if (Physics.Raycast(rayBackward, out hitInfo, (o - x[i]).magnitude)) {
                        hitBackward = true;
                        pointBackward = hitInfo.point;
                        normalBackward = hitInfo.normal;
                        distBackward = hitInfo.distance;
                    }

                    // position is outside, projected is inside
                    if (hitForward && !hitBackward) {
                        float dist = Vector3.Dot(pointForward - projectedX[i], normalForward);
                        collisionConstraints.Add(new NewMeshCollisionConstraint(i, normalForward, dist, triangleIndices));
                    }

                    // position is inside, projected is outside
                    if (hitForward && hitBackward) {
                        // Do nothing
                    }

                    // both is inside
                    if (!hitForward && hitBackward) {
                        Vector3 p = projectedX[i] - target.transform.position + new Vector3(extent, extent, extent);

                        p /= extent;
                        
                        var x = (int) (p.x * (resolution - 1)) / 2 + 1;
                        var y = (int) (p.y * (resolution - 1)) / 2 + 1;
                        var z = (int) (p.z * (resolution - 1)) / 2 + 1;

                        Color gradient = targetVolume.texture.GetPixel(x, y, z);
                        Vector3 normal = new Vector3(gradient.r, gradient.g, gradient.b).normalized;

                        Vector3 o2 = projectedX[i] + normal * 500.0f;
                        Ray closest = new Ray(o2, (projectedX[i] - o2).normalized);
                        if (Physics.Raycast(closest, out hitInfo, Mathf.Infinity)) {
                            hitForward = true;
                            pointForward = hitInfo.point;
                            normalForward = hitInfo.normal;
                            distForward = hitInfo.distance;
                        }
                        distForward = (o2 - projectedX[i]).magnitude - distForward;
                        distBackward = (o - projectedX[i]).magnitude - distBackward;

                        if (distForward < distBackward) {
                            float dist = Vector3.Dot(pointForward - projectedX[i], normalForward);
                            collisionConstraints.Add(new NewMeshCollisionConstraint(i, normalForward, dist, triangleIndices));
                        } else {
                            float dist = Vector3.Dot(pointBackward - projectedX[i], normalBackward);
                            collisionConstraints.Add(new NewMeshCollisionConstraint(i, normalBackward, dist, triangleIndices));
                        }
                        
                    }
                }
                
                if (collided) { 
                    ClothFrictionCollider frictionCollider = targetCollider.GetComponent<ClothFrictionCollider>();
                    if (frictionCollider != null) {
                        frictions[i] = Mathf.Min(frictions[i], 1f - frictionCollider.friction);
                    }
                }
        }
        //Debug.Log(counter);
    }


    private void AddDistanceConstraints() {
        // use a set to get unique edges
        HashSet<Edge> edgeSet = new HashSet<Edge>(new EdgeComparer());
        for (int i = 0; i < triangles.Length; i++) {
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[1]));
            edgeSet.Add(new Edge(triangles[i].vertices[0], triangles[i].vertices[2]));
            edgeSet.Add(new Edge(triangles[i].vertices[1], triangles[i].vertices[2]));
        };

        foreach (Edge e in edgeSet) {
            constraints.Add(new DistanceConstraint(e, positions, distanceCompressionStiffness, distanceStretchStiffness));
        }
    }


    private void AddBendingConstraints() {
        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
        foreach (Triangle tri in triangles) {
            Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
            if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri)) {
                wingEdges[e1].Add(tri);
            }
            else {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e1, tris);
            }

            Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
            if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri)) {
                wingEdges[e2].Add(tri);
            }
            else {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e2, tris);
            }

            Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
            if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri)) {
                wingEdges[e3].Add(tri);
            }
            else {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e3, tris);
            }
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        List<Edge> keyList = wingEdges.Keys.ToList();
        foreach (Edge e in keyList) {
            if (wingEdges[e].Count < 2) {
                wingEdges.Remove(e);
            }
        }

        foreach (Edge wingEdge in wingEdges.Keys) {
            /* wingEdges are indexed like in the Bridson,
             * Simulation of Clothing with Folds and Wrinkles paper
             *    3
             *    ^
             * 0  |  1
             *    2
             */

            int[] indices = new int[4];
            indices[2] = wingEdge.startIndex;
            indices[3] = wingEdge.endIndex;

            int b = 0;
            foreach (Triangle tri in wingEdges[wingEdge]) {
                for (int i = 0; i < 3; i++) {
                    int point = tri.vertices[i];
                    if (point != indices[2] && point != indices[3]) {
                        //tri #1
                        if (b == 0) {
                            indices[0] = point;
                            break;
                        }
                        //tri #2
                        else if (b == 1) {
                            indices[1] = point;
                            break;
                        }
                    }
                }
                b++;
            }

            if (bendingMethod == BendingMethod.isometricBending) {
                constraints.Add(new IsometricBendingConstraint(indices, positions, bendingStiffness));

            }
            else {
                constraints.Add(new BendingConstraint(indices, positions, bendingStiffness));
            }
        }
    }


    private void AddPointConstraints() {
        if (pointConstraintType == PointConstraintType.topCorners) {
            pointConstraints.Add(new PointConstraint(rows * (columns + 1)));
            pointConstraints.Add(new PointConstraint((rows + 1) * (columns + 1) - 1));
        }
        else if (pointConstraintType == PointConstraintType.topRow) {
            for (int i = 0; i <= columns; i++) {
                pointConstraints.Add(new PointConstraint(rows * (columns + 1) + i));
            }
        }
        else if (pointConstraintType == PointConstraintType.leftCorners) {
            pointConstraints.Add(new PointConstraint(0));
            pointConstraints.Add(new PointConstraint(rows * (columns + 1)));
        }
        else if (pointConstraintType == PointConstraintType.leftRow) {
            for (int i = 0; i <= rows; i++) {
                pointConstraints.Add(new PointConstraint(i * (columns + 1)));
            }
        }

        for (int i = 0; i < pointConstraintCustomIndices.Length; i++) {
            int index = pointConstraintCustomIndices[i];
            if (index >= 0 && index < numParticles) {
                pointConstraints.Add(new PointConstraint(index));
            }
        }
    }


    private void ApplyExplicitEuler(float dt) {
        for (int i = 0; i < numParticles; i++) {
            projectedPositions[i] = positions[i] + velocities[i] * dt;
        }
    }


    private void ApplyVerlet(float dt) {
        for (int i = 0; i < numParticles; i++) {
            projectedPositions[i] = positions[i] + velocities[i] * dt;
        }
    }


    private void SatisfyConstraints() {
        // randomly shuffle constraints
        constraints.Shuffle();
        collisionConstraints.Shuffle();

        // satisfy normal constraints first
        for (int i = 0; i < constraints.Count; i++) {
            constraints[i].Satisfy(projectedPositions, invMass);
        }

        // then satisfy collision constraints
        for (int i = 0; i < collisionConstraints.Count; i++) {
            collisionConstraints[i].Satisfy(projectedPositions, invMass);
        }
    }


    private void SatisfyPointConstraints(float dt) {
        for (int i = 0; i < pointConstraints.Count; i++) {
            pointConstraints[i].Satisfy(projectedPositions, positions);
        }
    }


    private void UpdateVertices(float dt) {
        for (int i = 0; i < numParticles; i++) {
            // step 13: velocity = (projectedPos - currentPos) / dt
            velocities[i] = (projectedPositions[i] - positions[i]) / dt;
            // step 14: currentPos = projectedPos
            positions[i] = projectedPositions[i];
        }
        for (int i = 0; i < triangles.Length; i++) {
            x[i] = projectedX[i];
            distances[i] = GetField(x[i]);
        }
    }


    private void ApplyFriction() {
        // do simple air resistance by reducing speed of every vertex
        for (int i = 0; i < numParticles; i++) {
            velocities[i] *= 0.998f;
        }
        // do simple ground friction by setting speed of objects on ground to zero
        for (int i = 0; i < numParticles; i++) {
            if (frictions[i] < 1) {
                velocities[i] *= frictions[i];
                //velocities[i] = new Vector3(Mathf.Abs(velocities[i][0]) < 0.2f ? 0 : velocities[i][0],
                //                            Mathf.Abs(velocities[i][1]) < 0.2f ? 0 : velocities[i][1],
                //                            Mathf.Abs(velocities[i][2]) < 0.2f ? 0 : velocities[i][2]);
            }
        }
    }

    // Distance to a sphere's surface
    private float SDFCircle(Vector3 a) {
        //float radius = targetCollider.GetComponent<SphereCollider>().radius * targetCollider.transform.lossyScale.x;
        float radius = targetBounds.x;

        float da = (a - target.transform.position).magnitude - radius;

        return Mathf.Clamp(da, -1.0f, 1.0f);
    }

    // Distance to a box's surface
    // Can be used for a bounding box or a cube mesh
    private float SDFBox(Vector3 a) {
        Vector3 p = a - target.transform.position;
        Vector3 targetBounds = target.GetComponent<Renderer>().bounds.size;
        
        float left = -targetBounds.x / 2, right = targetBounds.x / 2;
        float bottom = -targetBounds.y / 2, top = targetBounds.y / 2;
        float back = -targetBounds.z / 2, forward = targetBounds.z / 2;

        float dx = DistanceAux(p.x, - targetBounds.x / 2, targetBounds.x / 2);
        float dy = DistanceAux(p.y, - targetBounds.y / 2, targetBounds.y / 2);
        float dz = DistanceAux(p.z, - targetBounds.z / 2, targetBounds.z / 2);

        if (a.x > left && a.x < right && a.y > bottom && a.y < top && a.z > back && a.z < forward) {
            return Mathf.Min(dx, dy, dz);
        }
        return Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
    }


    private float DistanceAux(float x, float min, float max) {
        if (x < min) return min - x;
        else if (x > max) return x - max;
        else return 0.0f;
    }


    private float SearchDistance(Vector3 a) {
        if (targetVolume) {
            // Convert to local space
            if (Utility.IsPointInCube(a - target.transform.position, new Vector3(extent * 2, extent * 2, extent * 2))) {
                Vector3 p = a - target.transform.position + new Vector3(extent, extent, extent);
            
                p /= extent;
                
                var x = (int) (p.x * (resolution - 1)) / 2 + 1;
                var y = (int) (p.y * (resolution - 1)) / 2 + 1;
                var z = (int) (p.z * (resolution - 1)) / 2 + 1;

                Color gradient = targetVolume.texture.GetPixel(x, y, z);
                return gradient.a;
            } else {
                return 1.0f;
            }
        } else {
            return 1.0f;
        }
    }


    private float GetField(Vector3 a) {
        if (targetCollider) {
            if (targetCollider.GetType() == typeof(SphereCollider)) {
                return SDFCircle(a);
            } else if (targetCollider.GetType() == typeof(BoxCollider)) {
                return SDFBox(a);
            } else if (targetCollider.GetType() == typeof(MeshCollider)) {
                return SearchDistance(a);
            }
        }
        return 1.0f;
    }

    // Only used for Primitive shapes
    private Vector3 GetGradient(Vector3 a) {
        Vector3 gradient = Vector3.zero;
        float eps = 0.01f;

        gradient.x = GetField(a + Vector3.right * eps) - GetField(a + Vector3.left * eps);
        gradient.y = GetField(a + Vector3.up * eps) - GetField(a + Vector3.down * eps);
        gradient.z = GetField(a + Vector3.forward * eps) - GetField(a + Vector3.back * eps);
        gradient /= 2.0f * eps;

        return gradient;
    }

    // If target has volume data, see DFVolume
    private Vector3 GetVolumeData(Vector3 a) {
        if (targetVolume) {
            // Convert to local space
            if (Utility.IsPointInCube(a - target.transform.position, new Vector3(extent * 2, extent * 2, extent * 2))) {
                Vector3 p = a - target.transform.position + new Vector3(extent, extent, extent);
                p /= extent;
                
                var x = (int) (p.x * (resolution - 1)) / 2 + 1;
                var y = (int) (p.y * (resolution - 1)) / 2 + 1;
                var z = (int) (p.z * (resolution - 1)) / 2 + 1;

                Color gradient = targetVolume.texture.GetPixel(x, y, z);
                Vector4 vector = gradient;
                return vector;
            } else {
                return Vector3.zero;
            }
        } else {
            return Vector3.zero;
        }
    }


    int FetchIndex(int xi, int yi, int zi)
    {
        xi = Mathf.Clamp(xi, 0, resolution - 1);
        yi = Mathf.Clamp(yi, 0, resolution - 1);
        zi = Mathf.Clamp(zi, 0, resolution - 1);
        return xi + resolution * (yi + resolution * zi);
    }

    
    private void OnDrawGizmos() {
        /*
        if (Application.isPlaying) {
            for (int i = 0; i < triangles.Length; i++) {
                if (distances[i] <= 0.1f)  {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(projectedX[i], 0.1f);
                }
            }
        }
        */
    }
    
}