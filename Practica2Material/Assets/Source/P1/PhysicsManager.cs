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
    }

    private void SoftConstraintsStep()
    {
    }

    #endregion

}
