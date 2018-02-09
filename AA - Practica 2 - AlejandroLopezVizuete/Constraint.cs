using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic constraint which needs to be dropped on a sphere.
/// We only implement spherical joints. Other constraints would require a generalization of the constraint interface.
/// </summary>
public class Constraint : MonoBehaviour
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public Constraint()
    {
        this.m_manager = null;
    }

    #region EditorVariables

    public RigidBody BodyA;
    public RigidBody BodyB;

    #endregion

    #region OtherVariables

    private PhysicsManager m_manager;
    private int m_index; //Index of the constraint into the global vector of constraints
    private Vector3 m_pA; //Constraint point in the local reference frame of bodyA
    private Vector3 m_pB; //Constraint point in the local reference frame of bodyB
    // If BodyA or BodyB is not defined, m_pA or m_pB stores the global coordinates of the constraint point

    #endregion

    #region MonoBehaviour

    // Nothing to do here

    #endregion

    #region OtherMethods

    public void initialize(int index, PhysicsManager manager)
    {
        m_manager = manager;

        m_index = index;

        // Get the center of the sphere as the constraint point and transform it to the local frames of the bodies
        Transform xform = this.GetComponent<Transform>();

        if (xform != null)
        {
            Vector3 p = xform.localPosition;

            if (BodyA != null)
            {
                m_pA = BodyA.PointGlobalToLocal(p);
            }
            else
            {
                m_pA = p;
            }

            if (BodyB != null)
            {
                m_pB = BodyB.PointGlobalToLocal(p);
            }
            else
            {
                m_pB = p;
            }
        }
    }

    public int getSize()
    {
        return 3;
    }

    public void updateScene()
    {
        // Apply the average position to the mesh
        this.GetComponent<Transform>().position =
            0.5f * ((BodyA ? BodyA.PointLocalToGlobal(m_pA) : m_pA) + (BodyB ? BodyB.PointLocalToGlobal(m_pB) : m_pB));
    }

    public void addForces()
    {
        // TO BE COMPLETED: ADD SOFT CONSTRAINT FORCES TO THE RIGID BODIES

        //Creamos los Vector3 para almacenar las posiciones globales si existe el rigidbody
        Vector3 pA;
        Vector3 pB;

        //Si existe el rigidbody, cogemos las coordenadas globales del punto.
        //En caso contrario, la misma posicion local
        if (BodyA)
            pA = BodyA.PointLocalToGlobal(m_pA);
        else
            pA = m_pA;

        //Lo mismo para la zona B
        if (BodyB)
            pB = BodyB.PointLocalToGlobal(m_pB);
        else
            pB = m_pB;

        //Calculamos la fuerza elastica para ambos puntos considerando la restriccion
        Vector3 FelasticaA = m_manager.K * (pB - pA);
        Vector3 FelasticaB = m_manager.K * (pA - pB);

        //Si existe el objeto A, añadimos la fuerza elastica a su fuerza, y realizamos " (Rr) X Felastica " para el torque
        if (BodyA)
        {
            //Debug.Log("BODY-A");
            BodyA.m_force += FelasticaA;
            BodyA.m_torque += Vector3.Cross(BodyA.VecLocalToGlobal(BodyA.PointGlobalToLocal(pA)), FelasticaA);
        }

        //El mismo caso si existe el objeto B
        if (BodyB)
        { 
            //Debug.Log("BODY-B");
            BodyB.m_force += FelasticaB;
            BodyB.m_torque += Vector3.Cross(BodyB.VecLocalToGlobal(BodyB.PointGlobalToLocal(pB)), FelasticaB);
        }
    }

    public void getConstraintVector(VectorXD C0)
    {
        // TO BE COMPLETED: WRITE CONSTRAINT VALUES TO C0

        //Creamos los Vector3 para almacenar las posiciones globales si existe el rigidbody
        Vector3 pA;
        Vector3 pB;

        //Si existe el rigidbody, cogemos las coordenadas globales del punto.
        //En caso contrario, la misma posicion local
        if (BodyA)
            pA = BodyA.PointLocalToGlobal(m_pA);
        else
            pA = m_pA;

        //Lo mismo para la zona B
        if (BodyB)
            pB = BodyB.PointLocalToGlobal(m_pB);
        else
            pB = m_pB;

        //Realizamos la diferencia entre los puntos
        Vector3 restriccion = pA - pB;
        
        //Y los adjudicamos al vector C0
        C0[m_index] = restriccion.x;
        C0[m_index+1] = restriccion.y;
        C0[m_index+2] = restriccion.z;

    }

    public void getConstraintJacobian(MatrixXD J)
    {
        // TO BE COMPLETED: WRITE CONSTRAINT JACOBIANS TO J

        //Creamos la matriz Identidad para utilizarla en la matriz Jacobiana
        MatrixXD I = DenseMatrixXD.CreateIdentity(3);
        
        //Si existe un Rigidoby en la parte A del contrain
        if (BodyA)
        {
            //Añadimos la Identidad en la posicion (indice del contrain, indice del objeto)
            J.SetSubMatrix(m_index,BodyA.m_index,I);
            //Añadimos la Identidad en la posicion (indice del contrain, indice del objeto+3)
            J.SetSubMatrix(m_index,BodyA.m_index+3, -Utils.Skew(BodyA.VecLocalToGlobal(m_pA)));

            //De esta forma, ponemos cada Matrix dependiendo del indice de ambos elementos
        }

        // Si existe algun objeto en la parte B, realizamos lo mismo.
        if(BodyB)
        {
            J.SetSubMatrix(m_index, BodyB.m_index,-I);
            J.SetSubMatrix(m_index, BodyB.m_index+3, Utils.Skew(BodyB.VecLocalToGlobal(m_pB)));
        }
        
    }

    #endregion

}
