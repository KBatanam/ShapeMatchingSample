using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;

namespace Nambata.Chapter5.Scripts
{
    public class ShapeMatchingBody
    {
        // 頂点データ
        public int NumVertices { get; private set; }

        private readonly List<Vector3> _orgPos = new();
        private readonly List<Vector3> _currPos = new();
        private readonly List<Vector3> _newPos = new();
        private readonly List<Vector3> _goalPos = new();

        private readonly List<float> _mass = new();
        private readonly List<Vector3> _velocity = new();
        private readonly List<bool> _fixed = new();
        //
        
        // シミュレーション設定
        private Vector3 _gravity;
        private float _damping;
        private float _restitution;
        private float _stiffness;
        private float _deformation;
        private bool _volumeConservation;
        
        private Vector3 _simMin;
        private Vector3 _simMax;

        private bool _linearDeformation;
        //
        
        private Matrix<float> _matrixAtqqInv;
        
        public void SetParameters(
            Vector3 gravity,
            float damping,
            float restitution,
            bool linearDeformation,
            bool volumeConservation,
            float stiffness,
            float deformation,
            Vector3 simMin,
            Vector3 simMax)
        {
            _gravity = gravity;
            _damping = damping;
            _restitution = restitution;
            _linearDeformation = linearDeformation;
            _volumeConservation = volumeConservation;
            _stiffness = stiffness;
            _deformation = deformation;
            _simMin = simMin;
            _simMax = simMax;
        }
        
        public void AddVertex(Vector3 pos, float mass)
        {
            _orgPos.Add(pos);
            _currPos.Add(pos);
            _newPos.Add(pos);
            _goalPos.Add(pos);
            _mass.Add(mass);
            _velocity.Add(Vector3.zero);
            //_fixed.Add(mass <= 0.0f);
            _fixed.Add(false);
            
            NumVertices++;
        }

        public Vector3 GetVertexPosition(int index)
        {
            return _currPos[index];
        }

        public void FixVertex(int index, Vector3 pos)
        {
            if (_fixed[index])
            {
                return;
            }
            
            _currPos[index] = pos;
            _newPos[index] = pos;
            _velocity[index] = Vector3.zero;
            _fixed[index] = true;
        }
        
        public void UnfixVertex(int index)
        {
            _fixed[index] = false;
        }

        public void Step(float dt)
        {
            CalExternalForces(dt);
            ShapeMatch();
            Integrate(dt);
        }
        
        private void CalExternalForces(float dt)
        {
            for (var i = 0; i < NumVertices; i++)
            {
                if (_fixed[i])
                {
                    continue;
                }
                
                _velocity[i] *= _damping;
                _velocity[i] += _gravity * dt;
                _newPos[i] = _currPos[i] + _velocity[i] * dt;
                _goalPos[i] = _orgPos[i];
            }
            
            ApplyBounds(dt);
        }

        private void Integrate(float dt)
        {
            for (var i = 0; i < NumVertices; i++)
            {
                _velocity[i] = (_newPos[i] - _currPos[i]) / dt;
                _currPos[i] = _newPos[i];
            }
        }

        private void ComputeCenters(out Vector3 cm, out Vector3 cmOrg)
        {
            cm = Vector3.zero;
            cmOrg = Vector3.zero;
            var totalMass = 0f;

            for (var i = 0; i < NumVertices; i++)
            {
                var mass = _mass[i];
                if (_fixed[i])
                {
                    mass *= 30.0f;
                }
                totalMass += mass;
                cm += _newPos[i] * mass;
                cmOrg += _orgPos[i] * mass;
            }

            cm /= totalMass;
            cmOrg /= totalMass;
        }

        private Matrix<float> ComputeApq(Vector3 cm, Vector3 cmOrg)
        {
            var matrixApq = Matrix<float>.Build.Dense(3, 3);

            for (var i = 0; i < NumVertices; i++)
            {
                var p = _newPos[i] - cm;
                var q = _orgPos[i] - cmOrg;
                var mass = _mass[i];
                if (_fixed[i])
                {
                    mass *= 30.0f;
                }

                matrixApq[0, 0] += mass * p.x * q.x;
                matrixApq[0, 1] += mass * p.x * q.y;
                matrixApq[0, 2] += mass * p.x * q.z;
                
                matrixApq[1, 0] += mass * p.y * q.x;
                matrixApq[1, 1] += mass * p.y * q.y;
                matrixApq[1, 2] += mass * p.y * q.z;
                
                matrixApq[2, 0] += mass * p.z * q.x;
                matrixApq[2, 1] += mass * p.z * q.y;
                matrixApq[2, 2] += mass * p.z * q.z;
            }
            
            return matrixApq;
        }
        
        private Matrix<float> ComputeAqq(Vector3 cmOrg)
        {
            var matrixAqq = Matrix<float>.Build.Dense(3, 3);

            for (int i = 0; i < NumVertices; i++)
            {
                var q = _orgPos[i] - cmOrg;
                var m = _mass[i];

                matrixAqq[0,0] += m * q.x * q.x;
                matrixAqq[0,1] += m * q.x * q.y;
                matrixAqq[0,2] += m * q.x * q.z;

                matrixAqq[1,0] += m * q.y * q.x;
                matrixAqq[1,1] += m * q.y * q.y;
                matrixAqq[1,2] += m * q.y * q.z;

                matrixAqq[2,0] += m * q.z * q.x;
                matrixAqq[2,1] += m * q.z * q.y;
                matrixAqq[2,2] += m * q.z * q.z;
            }

            return matrixAqq;
        }


        private Matrix<float> ExtractRotation(Matrix<float> matrixApq)
        {
            var svd = matrixApq.Svd();
            var u = svd.U;
            var vt = svd.VT;

            var det = (u * vt).Determinant();
            if (det < 0)
            {
                for (var i = 0; i < 3; i++)
                {
                    u[i, 2] = -u[i, 2];
                }
            }

            return u * vt;
        }
        
        private void ApplyGoalPositions(Matrix<float> matrixFinal, Vector3 cm, Vector3 cmOrg)
        {
            for (var i = 0; i < NumVertices; i++)
            {
                if (_fixed[i]) continue;

                var q = _orgPos[i] - cmOrg;
                
                _goalPos[i] = new Vector3(
                    matrixFinal[0, 0] * q.x + matrixFinal[0, 1] * q.y + matrixFinal[0, 2] * q.z,
                    matrixFinal[1, 0] * q.x + matrixFinal[1, 1] * q.y + matrixFinal[1, 2] * q.z,
                    matrixFinal[2, 0] * q.x + matrixFinal[2, 1] * q.y + matrixFinal[2, 2] * q.z
                ) + cm;
                
                _newPos[i] += (_goalPos[i] - _newPos[i]) * _stiffness;
            }
        }

        private void ShapeMatch()
        {
            if (NumVertices <= 1)
            {
                return;
            }
            
            ComputeCenters(out var cm, out var cmOrg);
            
            var matrixApq = ComputeApq(cm, cmOrg);
            var matrixR = ExtractRotation(matrixApq);

            if (_linearDeformation)
            {
                var matrixAqq = ComputeAqq(cmOrg);
                var matrixA = matrixApq * matrixAqq.Inverse();

                if (_volumeConservation)
                {
                    float det = Mathf.Abs(matrixA.Determinant());
                    if (det > 0.0001f)
                    {
                        float detScale = Mathf.Pow(det, 1.0f / 3.0f);
                        matrixA /= detScale;
                    }
                }

                var matrixRL = (_deformation * matrixA) + ((1.0f - _deformation) * matrixR);
                ApplyGoalPositions(matrixRL, cm, cmOrg);
            }
            else
            {
                PerformQuadraticShapeMatch(cm, cmOrg, matrixR);
            }
        }

        private void PerformQuadraticShapeMatch(Vector3 cm, Vector3 cmOrg, Matrix<float> matrixR)
        {
            var matrixAtpq = Matrix<float>.Build.Dense(3, 9);

            if (_matrixAtqqInv == null)
            {
                //InitializeMatrixAtqqInv();
                var matrixAtqq =  Matrix<float>.Build.Dense(9, 9);
                for (var i = 0; i < NumVertices; i++)
                {
                    var mass = _mass[i];
                    var qt = ComputeTildeQVector(_orgPos[i] - cmOrg);
                    matrixAtqq += mass * qt.OuterProduct(qt);
                }
                _matrixAtqqInv = matrixAtqq.Inverse();
            }

            for (var i = 0; i < NumVertices; i++)
            {
                var mass  = _mass[i];
                if (_fixed[i])
                {
                    mass *= 30.0f;
                }
                
                var p = _newPos[i] - cm;
                var pVec = Vector<float>.Build.Dense(new[] {p.x, p.y, p.z});
                
                var qt = ComputeTildeQVector(_orgPos[i] - cmOrg);
                
                matrixAtpq += mass * pVec.OuterProduct(qt);
            }
            
            var matrixAt = matrixAtpq * _matrixAtqqInv;

            if (_volumeConservation)
            {
                var linearPart = matrixAt.SubMatrix(0, 3, 0, 3);
                var det = Mathf.Abs(linearPart.Determinant());
                if (det > 0.0001f)
                {
                    var detScale = Mathf.Pow(det, -1.0f / 3.0f);
                    matrixAt *= detScale;
                }
            }
            
            for (var i = 0; i < 3; i++)
            {
                for (var j = 0; j < 9; j++)
                {
                    matrixAt[i, j] *= _deformation;
                    if (j < 3)
                    {
                        matrixAt[i, j] += (1.0f - _deformation) * matrixR[i, j];
                    }
                }
            }

            ApplyQuadraticGoalPositions(matrixAt, cm, cmOrg);
        }

        private void ApplyQuadraticGoalPositions(Matrix<float> matrixAt, Vector3 cm, Vector3 cmOrg)
        {
            for (var i = 0; i < NumVertices; i++)
            {
                if (_fixed[i])
                {
                    continue;
                }
                
                var q = _orgPos[i] - cmOrg;
                var qt = ComputeTildeQVector(q);
                
                var goalRel = matrixAt * qt;
                
                _goalPos[i] = new Vector3(goalRel[0], goalRel[1], goalRel[2]) + cm;
                
                _newPos[i] += (_goalPos[i] - _newPos[i]) * _stiffness;
            }
        }
        
        private Vector<float> ComputeTildeQVector(Vector3 q)
        {
            return Vector<float>.Build.Dense(new [] {
                q.x, q.y, q.z,
                q.x * q.x, q.y * q.y, q.z * q.z,
                q.x * q.y, q.y * q.z, q.z * q.x
            });
        }

        private void ApplyBounds(float dt)
        {
            for (var i = 0; i < NumVertices; i++)
            {
                if (_fixed[i])
                {
                    continue;
                }
                
                var poss = _currPos[i];
                var newPoss = _newPos[i];
                var vel = _velocity[i];

                if (newPoss.x < _simMin.x || newPoss.x > _simMax.x)
                {
                    newPoss.x = poss.x - vel.x * dt * _restitution;
                    newPoss.y = poss.y;
                    newPoss.z = poss.z;
                }

                if (newPoss.y < _simMin.y || newPoss.y > _simMax.y)
                {
                    newPoss.y = poss.y - vel.y * dt * _restitution;
                    newPoss.z = poss.z;
                    newPoss.x = poss.x;
                }

                if (newPoss.z < _simMin.z || newPoss.z > _simMax.z)
                {
                    newPoss.z = poss.z - vel.z * dt * _restitution;
                    newPoss.x = poss.x;
                    newPoss.y = poss.y;
                }
                
                _newPos[i] = newPoss;
            }
        }
    }
}
