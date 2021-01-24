using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Natural : MonoBehaviour
{
    public int CONTROL_MODE;
    public MPControl mpc_1;
    public MPControl_2 mpc_2;
    public MPControl_3 mpc_3;
    public MPControl_4 mpc_4;
    public float TorqueConstant=1;
    public float GrabityConstant=9.81f;
    public float Mass;
    public GameObject Body;
    public GameObject PredictivePositionIndicaterSample;
    public GameObject PowerIndicaterLeft,PowerIndicaterRight;
    Transform BodyTransform;
    Rigidbody2D rb;
    Transform PowerIndicaterTransformLeft,PowerIndicaterTransformRight;
    float BodyPosition_x,BodyPosition_y;
    float RightPower,LeftPower;
    float Force_x,Force_y;
    GameObject[] PredictivePositionIndicater;
    Transform[] PredictivePositionIndicaterTransform;
    Vector3[] PositionIndicaterPosition;
    int PredictionTime;
    LineRenderer lineRenderer;
    const float DegToRad=Mathf.PI/180;

    // Start is called before the first frame update
    void Start()
    {
        PowerIndicaterTransformLeft=PowerIndicaterLeft.GetComponent<Transform>();
        PowerIndicaterTransformRight=PowerIndicaterRight.GetComponent<Transform>();
        switch(CONTROL_MODE){
            case 0:
                PredictionTime=mpc_1.PredictionTime;
                break;
                
            case 1:
                PredictionTime=mpc_2.PredictionTime;
                break;

            case 2:
                PredictionTime=mpc_3.PredictionTime;
                break;
                
            case 3:
                PredictionTime=mpc_4.PredictionTime;
                break;

            default:
                break;
        }
        PositionIndicaterPosition=new Vector3[PredictionTime];
        PredictivePositionIndicater=new GameObject[PredictionTime+1];
        PredictivePositionIndicaterTransform=new Transform[PredictionTime+1]; 
        BodyTransform=Body.GetComponent<Transform>();
        rb=Body.GetComponent<Rigidbody2D>();
        Mass=rb.mass;
        rb.gravityScale=GrabityConstant;
        for(int i=0;i<PredictionTime+1;i++){
            PredictivePositionIndicater[i]=Instantiate(PredictivePositionIndicaterSample); 
            PredictivePositionIndicaterTransform[i]=PredictivePositionIndicater[i].GetComponent<Transform>();
        }
        lineRenderer=gameObject.AddComponent<LineRenderer>();
        lineRenderer.positionCount=PredictionTime;
        lineRenderer.widthMultiplier=0.02f;
    }

    // Update is called once per frame
    void Update()
    {
        
        switch(CONTROL_MODE){
            case 0:
                PowerIndicaterTransformLeft.localScale=new Vector3(0.05f,mpc_1.LeftPower[0]/3,1);
                PowerIndicaterTransformLeft.localPosition=new Vector3(-0.45f,mpc_1.LeftPower[0]/3/2+0.5f,0);
                PowerIndicaterTransformRight.localScale=new Vector3(0.05f,mpc_1.RightPower[0]/3,1);
                PowerIndicaterTransformRight.localPosition=new Vector3(0.45f,mpc_1.RightPower[0]/3/2+0.5f,0);
                lineRenderer.SetPosition(0,new Vector3(mpc_1.BodyPosition_x[0],mpc_1.BodyPosition_y[0],-0.9f));
                for(int i=1;i<PredictionTime;i++){
                    PredictivePositionIndicaterTransform[i].position=new Vector3(mpc_1.BodyPosition_x[i],mpc_1.BodyPosition_y[i],-1f);
                    PositionIndicaterPosition[i]=new Vector3(PredictivePositionIndicaterTransform[i].position.x,PredictivePositionIndicaterTransform[i].position.y,-0.9f);
                    lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
                }
                float CurrentBodyAngle1=BodyTransform.localEulerAngles.z<180? BodyTransform.localEulerAngles.z: BodyTransform.localEulerAngles.z-360;
                rb.AddForce(new Vector2((LeftPower+RightPower)*Mathf.Sin(-CurrentBodyAngle1*DegToRad),(LeftPower+RightPower)*Mathf.Cos(CurrentBodyAngle1*DegToRad)));
                rb.AddTorque(TorqueConstant*(RightPower-LeftPower));
                break;
            
            case 1:
                PowerIndicaterTransformLeft.localScale=new Vector3(mpc_2.Force_x[0]/1,0.5f,1);
                PowerIndicaterTransformLeft.localPosition=new Vector3(mpc_2.Force_x[0]/1/2,0,0);
                PowerIndicaterTransformRight.localScale=new Vector3(0.05f,mpc_2.Force_y[0]/1,1);
                PowerIndicaterTransformRight.localPosition=new Vector3(0,mpc_2.Force_y[0]/1/2,0);
                lineRenderer.SetPosition(0,new Vector3(mpc_2.BodyPosition_x[0],mpc_2.BodyPosition_y[0],-0.9f));
                for(int i=1;i<PredictionTime;i++){
                    PredictivePositionIndicaterTransform[i].position=new Vector3(mpc_2.BodyPosition_x[i],mpc_2.BodyPosition_y[i],-1f);
                    PositionIndicaterPosition[i]=new Vector3(PredictivePositionIndicaterTransform[i].position.x,PredictivePositionIndicaterTransform[i].position.y,-0.9f);
                    lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
                }
                rb.AddForce(new Vector2(Force_x,Force_y));
                break;

            case 2:
                LeftPower=mpc_3.LeftPower[0];
                RightPower=mpc_3.RightPower[0];
                PowerIndicaterTransformLeft.localScale=new Vector3(0.05f,LeftPower/3,1);
                PowerIndicaterTransformLeft.localPosition=new Vector3(-0.45f,LeftPower/3/2+0.5f,0);
                PowerIndicaterTransformRight.localScale=new Vector3(0.05f,RightPower/3,1);
                PowerIndicaterTransformRight.localPosition=new Vector3(0.45f,RightPower/3/2+0.5f,0);
                lineRenderer.SetPosition(0,new Vector3(mpc_3.BodyPosition_x[0],mpc_3.BodyPosition_y[0],-0.9f));
                for(int i=1;i<PredictionTime;i++){
                    PredictivePositionIndicaterTransform[i].position=new Vector3(mpc_3.BodyPosition_x[i],mpc_3.BodyPosition_y[i],-1f);
                    PositionIndicaterPosition[i]=new Vector3(PredictivePositionIndicaterTransform[i].position.x,PredictivePositionIndicaterTransform[i].position.y,-0.9f);
                    lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
                }
                float CurrentBodyAngle2=BodyTransform.localEulerAngles.z<180? BodyTransform.localEulerAngles.z: BodyTransform.localEulerAngles.z-360;
                rb.AddForce(new Vector2((LeftPower+RightPower)*Mathf.Sin(-CurrentBodyAngle2*DegToRad),(LeftPower+RightPower)*Mathf.Cos(CurrentBodyAngle2*DegToRad)));
                rb.AddTorque(TorqueConstant*(RightPower-LeftPower));
                break;

            case 3:
                PowerIndicaterTransformLeft.localScale=new Vector3(0.05f,mpc_4.LeftForce/3,1);
                PowerIndicaterTransformLeft.localPosition=new Vector3(-0.45f,mpc_4.LeftForce/3/2+0.5f,0);
                PowerIndicaterTransformRight.localScale=new Vector3(0.05f,mpc_4.RightForce/3,1);
                PowerIndicaterTransformRight.localPosition=new Vector3(0.45f,mpc_4.RightForce/3/2+0.5f,0);
                float CurrentBodyAngle3=BodyTransform.localEulerAngles.z<180? BodyTransform.localEulerAngles.z: BodyTransform.localEulerAngles.z-360;
                lineRenderer.SetPosition(0,BodyTransform.position+new Vector3(-mpc_4.BodyPosition[0]*Mathf.Sin(CurrentBodyAngle3*DegToRad),mpc_4.BodyPosition[0]*Mathf.Cos(CurrentBodyAngle3*DegToRad),-1f));
                for(int i=1;i<PredictionTime;i++){
                    PredictivePositionIndicaterTransform[i].position=BodyTransform.position+new Vector3(-mpc_4.BodyPosition[i]*Mathf.Sin(CurrentBodyAngle3*DegToRad),mpc_4.BodyPosition[i]*Mathf.Cos(CurrentBodyAngle3*DegToRad),-1f);
                    PositionIndicaterPosition[i]=new Vector3(PredictivePositionIndicaterTransform[i].position.x,PredictivePositionIndicaterTransform[i].position.y,-0.9f);
                    lineRenderer.SetPosition(i,PositionIndicaterPosition[i]);
                }
                rb.AddForce(new Vector2((LeftPower+RightPower)*Mathf.Sin(-CurrentBodyAngle3*DegToRad),(LeftPower+RightPower)*Mathf.Cos(CurrentBodyAngle3*DegToRad)));
                rb.AddTorque(TorqueConstant*(mpc_4.RightForce-mpc_4.LeftForce));
                break;
            
            default:
                break;
            
        }
    }
}
