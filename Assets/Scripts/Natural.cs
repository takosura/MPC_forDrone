using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{
    public MPControl mpc_control;
    public float TorqueConstant=1;
    public float GrabityConstant=9.81f;
    public float Mass;
    public GameObject Body;
    public GameObject PredictivePositionIndicaterSample;
    Transform BodyTransform;
    Rigidbody rb;
    Transform PowerIndicaterTransform;
    public GameObject PowerIndicater;
    float[] RightPower,LeftPower;
    GameObject[] PredictivePositionIndicater;
    Transform[] PredictivePositionIndicaterTransform;
    int PredictionTime;
    // Start is called before the first frame update
    void Start()
    {
        PredictionTime=mpc_control.PredictionTime;
        PredictivePositionIndicater=new GameObject[PredictionTime+1];
        PredictivePositionIndicaterTransform=new Transform[PredictionTime+1]; 
        BodyTransform=Body.GetComponent<Transform>();
        rb=Body.GetComponent<Rigidbody>();
        Mass=rb.mass;
        for(int i=0;i<PredictionTime+1;i++){
            PredictivePositionIndicater[i]=Instantiate (PredictivePositionIndicaterSample); 
            PredictivePositionIndicaterTransform[i]=PredictivePositionIndicater[i].GetComponent<Transform>();
        }
        //presentRightPower=mpc_control.
    }

    // Update is called once per frame
    void Update()
    {
    }
}
