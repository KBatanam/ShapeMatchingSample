using UnityEngine;

namespace Nambata.Chapter5.Scripts
{
    public class ShapeMatchingInteractor : MonoBehaviour
    {
        [SerializeField] private float maxPickDistance = 0.5f;

        private ShapeMatchingBehaviour _targetBehaviour;
        private int _pickedIndex = -1;
        private float _pickedDepth;
        private Camera _mainCamera;

        private void Start()
        {
            if (Camera.main != null)
            {
                _mainCamera = Camera.main;
            }
            
            if (_targetBehaviour == null)
            {
                _targetBehaviour = GetComponent<ShapeMatchingBehaviour>();
            }
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                TryPick();
            }

            if (Input.GetMouseButton(0) && _pickedIndex != -1)
            {
                Drag();
            }

            if (Input.GetMouseButtonUp(0))
            {
                Release();
            }
        }

        private void TryPick()
        {
            if (_mainCamera == null)
            {
                return;
            }

            var body = _targetBehaviour.GetBody();
            var ray = _mainCamera.ScreenPointToRay(Input.mousePosition);
            var camPos = _mainCamera.transform.position;

            var bestScore = float.MaxValue; 
            _pickedIndex = -1;
            
            var normals = _targetBehaviour.GetMeshNormals(); 

            for (var i = 0; i < body.NumVertices; i++)
            {
                var vPos = body.GetVertexPosition(i);
                var vNormal = normals[i];
                
                var viewDir = (vPos - camPos).normalized;
                var dot = Vector3.Dot(vNormal, viewDir);

                if (dot > 0.1f)
                {
                    continue;
                } 
                
                var relPos = vPos - ray.origin;
                var distToRay = Vector3.Cross(ray.direction, relPos).magnitude;
                var depth = Vector3.Dot(relPos, ray.direction);

                if (depth <= 0 || distToRay > maxPickDistance)
                {
                    continue;
                }
                
                var score = distToRay + (depth * 0.05f); 

                if (score < bestScore)
                {
                    bestScore = score;
                    _pickedIndex = i;
                    _pickedDepth = depth;
                }
            }
        }

        private void Drag()
        {
            if (_pickedIndex == -1 || _mainCamera == null)
            {
                return;
            }
    
            var ray = _mainCamera.ScreenPointToRay(Input.mousePosition);
            var targetPos = ray.origin + ray.direction * _pickedDepth;
            
            _targetBehaviour.GetBody().FixVertex(_pickedIndex, targetPos);
        }

        private void Release()
        {
            if (_pickedIndex != -1)
            {
                _targetBehaviour.GetBody().UnfixVertex(_pickedIndex);
                _pickedIndex = -1;
            }
        }
    }
}