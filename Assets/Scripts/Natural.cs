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
        PowerIndicaterTransformLeft=PowerIndicaterLeft.GetComponent<Transform>();
        PowerIndicaterTransformRight=PowerIndicaterRight.GetComponent<Transform>();
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
        PowerIndicaterTransformLeft.localScale=new Vector3(0.05f,LeftPower/10,1);
        PowerIndicaterTransformLeft.localPosition=new Vector3(-0.45f,LeftPower/10/2+0.5f,0);
        PowerIndicaterTransformRight.localScale=new Vector3(0.05f,RightPower/10,1);
        PowerIndicaterTransformRight.localPosition=new Vector3(0.45f,RightPower/10/2+0.5f,0);
        for(int i=1;i<PredictionTime;i++)PredictivePositionIndicaterTransform[i].position=new Vector3(mpc_control.BodyPosition_x[i],mpc_control.BodyPosition_y[i],-1f);
    }
}
