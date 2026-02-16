using UnityEngine;

namespace Nambata.Chapter5.Scripts
{
    public class ShapeMatchingBehaviour : MonoBehaviour
    {
        private enum DeformationMode
        {
            Linear,
            Quadratic
        }
        
        private Mesh _mesh;
        private ShapeMatchingBody _body;
        private Vector3[] _vertices;
        
        // シミュレーション設定
        [Header("Simulation Settings")]
        [SerializeField] private float timeStep = 1.0f / 60.0f;
        [SerializeField] private Vector3 gravity = new(0, -9.8f, 0);
        [Range(0.9f, 1.0f)]
        [SerializeField] private float damping = 0.99f;
        [Range(0f, 1f)]
        [SerializeField] private float restitution = 0.5f;
        [Range (0.1f, 10.0f)]
        [SerializeField] private float vertexMass = 1.0f;
        
        [Header("Shape Matching Settings")]
        [SerializeField] private DeformationMode deformationMode = DeformationMode.Linear;
        [SerializeField] private bool volumeConservation = true;
        [Range(0f, 1f)]
        [SerializeField] private float stiffness = 0.15f;
        [Range(0f, 1f)]
        [SerializeField] private float deformation = 0.15f;

        [Header("Bounds")]
        [SerializeField] private Vector3 simMin = new (-10.0f, 0.0f, -10.0f);
        [SerializeField] private Vector3 simMax = new (10.0f, 20.0f, 10.0f);
        //

        private void Start()
        {
            _mesh = GetComponent<MeshFilter>().mesh;
            _body = new ShapeMatchingBody();
            _vertices = _mesh.vertices;

            foreach (var vertex in _vertices)
            {
                Vector3 worldPos = transform.TransformPoint(vertex);
                _body.AddVertex(worldPos, vertexMass);
            }
            
            ApplyParameters();
        }

        private void OnValidate()
        {
            if (_body == null)
            {
                return;
            }
            
            ApplyParameters();
        }

        private void Update()
        {
            _body.Step(timeStep);
            
            for (int i = 0; i < _vertices.Length; i++)
            {
                _vertices[i] = transform.InverseTransformPoint(_body.GetVertexPosition(i));
            }
            
            _mesh.vertices = _vertices;
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
        }

        private void ApplyParameters()
        {
            _body.SetParameters(
                gravity,
                damping,
                restitution,
                deformationMode == DeformationMode.Linear,
                volumeConservation,
                stiffness,
                deformation,
                simMin,
                simMax
            );
        }

        public ShapeMatchingBody GetBody() => _body;

        public Vector3[] GetMeshNormals()
        {
            return _mesh.normals;
        }
    }
}