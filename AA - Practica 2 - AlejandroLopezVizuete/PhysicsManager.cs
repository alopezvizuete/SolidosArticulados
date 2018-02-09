using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;


/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		this.Paused = true;
		this.OneStep = false;
		this.TimeStep = 0.01f;
		this.Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
        this.MethodConstraints = ConstraintMethod.Hard;
	}

	/// <summary>
	/// Constraint method.
	/// </summary>
	public enum ConstraintMethod
	{
		Soft = 0,
		Hard = 1
	};

    #region InEditorVariables

	public bool Paused;
	public bool OneStep;
	public float TimeStep;
	public Vector3 Gravity;
    public float K;
    public float Damping;
    public List<GameObject> SimulableObjects;
    public List<GameObject> Constraints;
    public ConstraintMethod MethodConstraints;

	#endregion

	#region OtherVariables

	private List<ISimulable> m_objs;
    private List<Constraint> m_constraints;
    private int m_numdofs;
    private int m_numcs;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        int index = 0;

        m_objs = new List<ISimulable>(SimulableObjects.Capacity);

        foreach (GameObject obj in SimulableObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();

            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable model
                simobj.initialize(index, this);

                // Retrieve pos and vel size
                index += simobj.getNumDof();
            }
        }

        m_numdofs = index;

        index = 0;

        m_constraints = new List<Constraint>(Constraints.Capacity);

        foreach (GameObject obj in Constraints)
        {
            Constraint constraint = obj.GetComponent<Constraint>();

            if (constraint != null)
            {
                m_constraints.Add(constraint);

                // Initialize constraint
                constraint.initialize(index, this);

                // Retrieve constraint size
                index += constraint.getSize();
            }
        }

        m_numcs = index;
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.O))
			this.OneStep = !this.OneStep;

		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;
	}

	public void FixedUpdate () 
	{
		if (this.Paused && !this.OneStep)
			return; // Not simulating

		if (this.OneStep) // One!
			this.OneStep = false;

        // Integration method
        switch (MethodConstraints)
        {
            case ConstraintMethod.Hard:
                HardConstraintsStep();
                break;
            case ConstraintMethod.Soft:
            default:
                SoftConstraintsStep();
                break;
        }

        // Update visual elements
        foreach (ISimulable obj in m_objs)
        {
            obj.updateScene();
        }

        foreach (Constraint constraint in m_constraints)
        {
            constraint.updateScene();
        }
    }

    #endregion

    #region OtherMethods

    private void HardConstraintsStep()
    {
        //Creamos los vectores y matrices necesarios para el ejercicio
        VectorXD velocidad = new DenseVectorXD(m_numdofs);
        VectorXD fuerza = new DenseVectorXD(m_numdofs);
        VectorXD C0 = new DenseVectorXD(m_numcs);

        MatrixXD Masa = new DenseMatrixXD(m_numdofs, m_numdofs);
        MatrixXD J = new DenseMatrixXD(m_numcs,m_numdofs);

        //Para cada objeto, actualizamos las fuerzas en su rigidbody
        for (int i = 0; i < m_objs.Count; i++)
        {
            m_objs[i].addForcesAndMatrices();
        }
        //Por cada constrain, actualizamos sus fuerzas elasticas
        for (int i = 0; i < m_constraints.Count; i++)
        {
            m_constraints[i].addForces();
        }
        //Por cada objeto, actualizasmos las variables velocidad, fuerza y masa necesarias para el ejercicio
        for (int i = 0; i < m_objs.Count; i++)
        {
            m_objs[i].getVelocityVector(velocidad);
            m_objs[i].getForceVector(fuerza);
            m_objs[i].getMassMatrix(Masa);
        }
        //Por cada constrain, actualizamos las variables C0 y J para el ejercicio
        for (int i = 0; i < m_constraints.Count; i++)
        {
            m_constraints[i].getConstraintVector(C0);
            m_constraints[i].getConstraintJacobian(J);
        }
        //Creamos el gran sistema para resolver el Solve
        //Creamos las variables necesarias para el sistema grande
        MatrixXD A = Masa;
        VectorXD b = Masa * velocidad + TimeStep * fuerza;
        MatrixXD Jt = J.Transpose();

        //Creamos la Matriz grande del sistema, añadiendo A, Jacobiana y la traspuesta de la jacobiana
        MatrixXD MA = new DenseMatrixXD(m_numcs+m_numdofs, m_numcs+m_numdofs);
        MA.SetSubMatrix(0, 0, A);
        MA.SetSubMatrix(0, m_numdofs, Jt);
        MA.SetSubMatrix(m_numdofs, 0, J);

        //Creamos el vector grande del sistema, añadiendo b y el vector de restricciones
        VectorXD MB = new DenseVectorXD(m_numcs + m_numdofs);
        MB.SetSubVector(0,m_numdofs,b);
        MB.SetSubVector(m_numdofs, m_numcs, -(1/TimeStep)*C0);

        //Por ultimo, creamos el vector resultante, y guardamos el resultado del solve
        VectorXD MV = new DenseVectorXD(m_numcs + m_numdofs);
        MV = MA.Solve(MB);

        //Cogemos los valores de la velocidad del vector resultante, y actualizamos la velocidad
        //Segun el metodo simplectico
        MV.CopySubVectorTo(velocidad, 0, 0, m_numdofs);
        velocidad = (velocidad + TimeStep * fuerza * Masa.Inverse());

        //Añadimos los valores nuevos de velocidad y posicion a cada objeto
        for (int i = 0; i < m_objs.Count; i++)
        {
            m_objs[i].setVelocityVector(velocidad);
            m_objs[i].advancePosition();
        }

    }

    private void SoftConstraintsStep()
    {
        //Creamos los vectores y matrices necesarios para el ejercicio
        VectorXD velocidad = new DenseVectorXD(m_numdofs);
        VectorXD fuerza = new DenseVectorXD(m_numdofs);
        MatrixXD Masa = new DenseMatrixXD(m_numdofs, m_numdofs);

        //Para cada objeto, actualizamos las fuerzas en su rigidbody
        for (int i = 0; i < m_objs.Count; i++)
        {
            //Debug.Log("OBJS");
            m_objs[i].addForcesAndMatrices();
        }
        //Por cada constrain, actualizamos sus fuerzas elasticas
        for (int i = 0; i < m_constraints.Count; i++)
        {
            //Debug.Log("Constrains");
            m_constraints[i].addForces();
        }

        //Por cada objeto, actualizasmos las variables velocidad, fuerza y masa necesarias para el ejercicio
        for (int i = 0; i < m_objs.Count; i++)
        {
            m_objs[i].getVelocityVector(velocidad);
            m_objs[i].getForceVector(fuerza);
            m_objs[i].getMassMatrix(Masa);

        }

        //Actualizamos la velocidad segun el metodo simpletico
        velocidad = (velocidad + TimeStep * fuerza * Masa.Inverse());

        //Añadimos los valores nuevos de velocidad y posicion a cada objeto
        for (int i = 0; i < m_objs.Count; i++)
        {
            m_objs[i].setVelocityVector(velocidad);
            m_objs[i].advancePosition();
        }


    }

    #endregion

}
