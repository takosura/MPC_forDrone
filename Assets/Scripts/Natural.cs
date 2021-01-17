using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{
    public MPControl mpc_control;
    public float TorqueConstant=1;
    public float GrabityConstant=9.81f;
    public GameObject Body;
    public GameObject PredictivePositionIndicaterSample;
    public GameObject PowerIndicaterLeft,PowerIndicaterRight;
    Transform BodyTransform;
    Transform PowerIndicaterTransformLeft,PowerIndicaterTransformRight;
    float BodyPosition_x,BodyPosition_y;
    float RightPower,LeftPower;
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
        for(int i=0;i<PredictionTime+1;i++){
            PredictivePositionIndicater[i]=Instantiate(PredictivePositionIndicaterSample); 
            PredictivePositionIndicaterTransform[i]=PredictivePositionIndicater[i].GetComponent<Transform>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        LeftPower=mpc_control.LeftPower[0];
        RightPower=mpc_control.RightPower[0];
        BodyPosition_x=mpc_control.BodyPosition_x[0];
        BodyPosition_y=mpc_control.BodyPosition_y[0];
        for(int i=1;i<PredictionTime;i++)PredictivePositionIndicaterTransform[i].position=new Vector3(BodyPosition_x,BodyPosition_y,-0.1f);
    }
}
