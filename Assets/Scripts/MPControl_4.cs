
using System.Collections;
using System.Collections.Generic; 
using UnityEngine;
//using DifiningObjects_MPControl;

/*
    前置き
    このプログラムは機体の位置を原点とし、機体の位置と目標位置をつないだ線分に重なるような座標系を考え、その座標系での入力を計算し、その後元の座標系に変換している
    このプログラムでは弧度法を用いて表現している
    コメントアウト機能を利用して各式の(全く不十分な)説明を行っているが、その時に利用する文字等について先に示しておく
    t:そのループ時の時刻
    τ:予測範囲での時刻、時刻tのときτ=0である
    d~:微小な~ ex:dx/dtはxの時間微分表す
    ð~:ex:ðx/ðuはxのuによる偏微分を表す
    x[t]:時刻tにおける目標物の位置
    v[t]:時刻tにおける目標物の速度
    u[t]:時刻tにおける入力

    パラメータ記録
    -2021/19/Jan(非ホロノミック的性質よりxは収束しないがyとthetaは上手いこと言った、入力も振動せず)
    PositionReference_x 0
    PositionReference_y 3
    GMRES_RepeatTime 2
    PredictionTime 10
    SstaleConstant 100
    YConstant 3
    YCOnstant_Stage 1
    ThetaConstant 3
    ThetaConstant_Stage 1
    LForceConsant_Stage 0.5
    RForceConsant_Stage 0.5
    その他 0
*/

public class MPControl_4 : MonoBehaviour{
    public float PositionReference_x=0;
    public float PositionReference_y=0;
    public Natural natural;
    public int GMRES_RepeatTime=2;
    public int PredictionTime=10;
    public float StableConstant=100;
    GameObject Body;
    public float XConstant,dXConstant,XConstant_Stage,dXConstant_Stage,
                                ThetaConstant_Stage,dThetaConstant_Stage, ForceConstant_Stage,TorqueConstant_Stage;
    public float FinalEvaluarionScope=0.5f;//[s]
    public float[] BodyPosition;//converted X
    public float[] BodyTheta;
    public float maxTh=Mathf.PI/4,maxLF=1,maxRF=1;
    Transform BodyTransform;
    float[] BodyVelocity;
    float[] BodyRev;//=d(BodyAngle)/dt
    public float LeftForce,RightForce;
    public float[] Force,Torque,DammyTheta,DammyLForce,DammyRForce;
    float[] DiffForce,DiffTorque,DiffDammyTheta,DiffDammyLForce,DiffDammyRForce;
    float[] DiffLagTh,DiffLagLF,DiffLagRF;    
    float[] AdjointVector_x,AdjointVector_dx,AdjointVector_theta,AdjointVector_dtheta;
    float[] Lag_theta,Lag_LForce,Lag_RForce;
    float PreviousTargetPosition,PreviousBodyTheta;
    float TorqueConstent;
    float InitialTime;
    float GrabityConstant;
    float Mass;
    const float DegToRad=Mathf.PI/180;
    int control_mode;

    // Start is called before the first frame update
    void Start(){
        BodyPosition=new float[PredictionTime+1];
        BodyVelocity=new float[PredictionTime+1];
        BodyTheta=new float[PredictionTime+1];
        BodyRev=new float[PredictionTime+1];
        Force=new float[PredictionTime+1];
        Torque=new float[PredictionTime+1];
        DammyTheta=new float[PredictionTime+1];
        DammyLForce=new float[PredictionTime+1];
        DammyRForce=new float[PredictionTime+1];
        DiffForce=new float[PredictionTime+1]; 
        DiffTorque=new float[PredictionTime+1]; 
        DiffDammyLForce=new float[PredictionTime+1];
        DiffDammyRForce=new float[PredictionTime+1];
        DiffDammyTheta=new float[PredictionTime+1];
        DiffLagTh=new float[PredictionTime+1];
        DiffLagLF=new float[PredictionTime+1];
        DiffLagRF=new float[PredictionTime+1];
        AdjointVector_x=new float[PredictionTime+1]; 
        AdjointVector_dx=new float[PredictionTime+1]; 
        AdjointVector_theta=new float[PredictionTime+1]; 
        AdjointVector_dtheta=new float[PredictionTime+1];
        Lag_LForce=new float[PredictionTime+1];
        Lag_RForce=new float[PredictionTime+1];
        Lag_theta=new float[PredictionTime+1];
        Body=natural.Body;
        BodyTransform=Body.GetComponent<Transform>();
        TorqueConstent=natural.TorqueConstant;
        GrabityConstant=natural.GrabityConstant;
        control_mode=natural.CONTROL_MODE;
        Mass=natural.Mass;

        for(int i = 0; i <PredictionTime+1 ; i++){
            DiffForce[i]=0; 
            DiffTorque[i]=0; 
            DiffDammyLForce[i]=0; 
            DiffDammyRForce[i]=0; 
            DiffDammyTheta[i]=0; 
            DiffLagTh[i]=0;
            DiffLagRF[i]=0;
            DiffLagTh[i]=0;
            Force[i]=0;
            Torque[i]=0;
            DammyLForce[i]=0;
            AdjointVector_x[i]=0; 
            AdjointVector_dx[i]=0; 
            AdjointVector_theta[i]=0; 
            AdjointVector_dtheta[i]=0; 
            BodyPosition[i]=0; 
            BodyVelocity[i]=0;
            BodyTheta[i]=0;
            BodyRev[i]=0;
        }  

        BodyPosition[0]=BodyTransform.position.x; 
        BodyVelocity[0]=0;
        BodyTheta[0]=0;
        BodyRev[0]=0; 
        LeftForce=Mass*GrabityConstant/2; 
        RightForce=Mass*GrabityConstant/2;
        Force[0]=LeftForce+RightForce;
        Torque[0]=0;
        for(int i=1;i<PredictionTime;i++){
            Force[i]=Force[0];
            Torque[i]=Torque[0];
            DammyLForce[i]=DammyLForce[0];
        }
        PreviousTargetPosition=BodyPosition[0];  
        PreviousBodyTheta=BodyTheta[0];
        InitialTime=Time.time;
    }

    

    // Update is called once per frame
    void FixedUpdate(){
        // モデルへの入力はnaturalクラスでおこなっている

        if(control_mode==3){

            //measure real delta time (not Eval delta time)
            //制御ループ周期(dt)測定
            float dt=Time.deltaTime; 

            //meature present Object's position and velocity and input them into ObjectPosition[0],ObjectVelocity[0]
            //目標物の位置と速度を計測し、x[τ=0],v[τ=0]に代入する
            float TargetPosition=Mathf.Sqrt(Mathf.Pow(BodyTransform.position.x-PositionReference_x,2)+Mathf.Pow(BodyTransform.position.y-PositionReference_y,2));
            float TargetDirectionAngle=Mathf.Atan2(PositionReference_x-BodyTransform.position.x,PositionReference_y-BodyTransform.position.y);
            BodyTheta[0]= -BodyTransform.localEulerAngles.z*DegToRad<Mathf.PI? -BodyTransform.localEulerAngles.z*DegToRad: -BodyTransform.localEulerAngles.z*DegToRad+Mathf.PI*2;
            BodyTheta[0]=TargetDirectionAngle-BodyTheta[0];
            BodyRev[0]=(BodyTheta[0]-PreviousBodyTheta)/dt;
            BodyPosition[0]=0; 
            BodyVelocity[0]=-(TargetPosition-PreviousTargetPosition)/dt; 
            PreviousTargetPosition=TargetPosition;
            PreviousBodyTheta=BodyTheta[0];

            //difine EvalTime in this loop
            //EvalutionTime will converge to FinalEvalScope/PredictionTime
            //予測範囲を予測範囲分割数で割った予測空間内でのループ周期(dτ)を設定する
            //予め設定した最終予測範囲に収束するように設定する：ループ開始時は予測の精度が高くないため予測範囲は初めは0で0→最終予測範囲となるように
            float EvalDT=FinalEvaluarionScope*(1-Mathf.Exp(-2.5f*(Time.time-InitialTime+0.01f)))/PredictionTime; 

            //forsee ObjectPosition[i] and ObjectVelocity[i] using ObjectPosition[0] and ObjectVelocity[0], given InputPosition[i]
            //上で与えられたv[τ=0],x[τ=0](つまりv[t],x[t])とuからx[i],v[i]を順に予想していく
            for(int i=1;i<PredictionTime+1; i++) {
                BodyPosition[i]=BodyPosition[i-1]+BodyVelocity[i-1]*EvalDT;
                BodyVelocity[i]=BodyVelocity[i-1]+Force[i-1]*EvalDT;
                BodyTheta[i]=BodyTheta[i-1]+BodyRev[i-1]*EvalDT;
                BodyRev[i]=BodyRev[i-1]+TorqueConstent*Torque[i-1]*EvalDT;
            }

            //calculate AdjointVector[i,0]and[i,1] :[i,0] for position, [i,1] for velocity
            //随伴変数を求める
            //at first, AdjointVector[last] is calculated by AdjointVector[last]=ð(TerminalCost)/ðx
            //初めに、予測範囲内で最終の随伴変数を求める、これは終端コストのx[N*dτ]での偏微分に等しい
            AdjointVector_x[PredictionTime]=XConstant*(BodyPosition[PredictionTime]-TargetPosition); 
            AdjointVector_dx[PredictionTime]=dXConstant*BodyVelocity[PredictionTime];
            AdjointVector_theta[PredictionTime]=0; 
            AdjointVector_dtheta[PredictionTime]=0;

            //following AdjointVector[last], AdjointVector[last -1] can be calculated by AdjointVector[last -1]=AdjointVector[last]+ ðH/ðx*dτ, and so on.
            //逆順に随伴変数を求めていく。AdjointVector[i-1]=AdjointVector[i]+ ðH/ðx*dτのように求められる。
            for(int i=PredictionTime-1;i>0;i--){ 
                AdjointVector_x[i]=AdjointVector_x[i+1]+XConstant_Stage*(BodyPosition[i]-TargetPosition)*EvalDT;
                AdjointVector_dx[i]=AdjointVector_dx[i+1]+(dXConstant_Stage*BodyVelocity[i]+AdjointVector_x[i+1])*EvalDT;
                AdjointVector_theta[i]=AdjointVector_theta[i+1]+(ThetaConstant_Stage*(BodyTheta[i]-TargetDirectionAngle)
                                            +AdjointVector_theta[i+1]+2*Lag_theta[i+1]*(BodyTheta[i]-TargetDirectionAngle)
                                            +Lag_LForce[i]*(-(Force[i]/Mathf.Cos(BodyTheta[i])-Torque[i]-maxLF)*Force[i]*Mathf.Sin(BodyTheta[i])/2/Mathf.Cos(BodyTheta[i])/Mathf.Cos(BodyTheta[i]))
                                            +Lag_RForce[i]*(-(Force[i]/Mathf.Cos(BodyTheta[i])+Torque[i]-maxRF)*Force[i]*Mathf.Sin(BodyTheta[i])/2/Mathf.Cos(BodyTheta[i])/Mathf.Cos(BodyTheta[i])))*EvalDT;
                AdjointVector_dtheta[i]=AdjointVector_dtheta[i+1]+(dThetaConstant_Stage*BodyRev[i]+AdjointVector_theta[i+1])*EvalDT;
            }

            //calculate dU/dt using GMRES method
            float[] Difference_Force=new float[PredictionTime];
            float[] Difference_Torque=new float[PredictionTime];
            float[] Difference_DammyTheta=new float[PredictionTime];
            float[] Difference_DammyLForce=new float[PredictionTime];
            float[] Difference_DammyRForce=new float[PredictionTime];
            float[] Difference_ConstraintLF=new float[PredictionTime];
            float[] Difference_ConstraintRF=new float[PredictionTime];
            float[] Difference_ConstraintTh=new float[PredictionTime];
            float[] Difference=new float[PredictionTime*8];
            float DifferenceInnerProduct=0; 
            float[,] OrthoBasis=new float[GMRES_RepeatTime+1, PredictionTime*8]; 

            for(int i=0;i<PredictionTime;i++) {
                float F_Force(float force,float torque,float Lag_LForce,float Lag_RForce) {
                    return ForceConstant_Stage*force+AdjointVector_dx[i+1]/Mass+Lag_LForce*(force/Mathf.Cos(BodyTheta[i])-torque-maxLF)/2/Mathf.Cos(BodyTheta[i])
                                                                                     +Lag_RForce*(force/Mathf.Cos(BodyTheta[i])+torque-maxRF)/2/Mathf.Cos(BodyTheta[i]);
                }
                float F_Torque(float force,float torque,float Lag_LForce,float Lag_RForce) {
                    return TorqueConstant_Stage*torque+TorqueConstent*AdjointVector_dtheta[i+1]
                                    -Lag_LForce*(force/Mathf.Cos(BodyTheta[i])-torque-maxLF)/2-Lag_RForce*(force/Mathf.Cos(BodyTheta[i])+torque-maxRF)/2;
                }
                float F_DammyTheta(float dammyTheta,float lag_theta) {
                    return -0.01f+2*lag_theta*dammyTheta;
                }
                float F_DammyLForce(float dammyLForce,float lag_LForce){
                    return -0.01f+2*lag_LForce*dammyLForce;
                }
                float F_DammyRForce(float dammyRForce,float lag_RForce){
                    return -0.01f+2*lag_RForce*dammyRForce;
                }
                float F_CosntraintTh(float dammyTheta){
                    return Mathf.Pow(BodyTheta[i]-TargetDirectionAngle,2)+Mathf.Pow(dammyTheta,2)-Mathf.Pow(maxTh,2);
                }
                float F_CosntraintLF(float force,float torque,float dammyLForce){
                    return Mathf.Pow((force/Mathf.Cos(BodyTheta[i])-torque-maxLF)/2,2)+Mathf.Pow(dammyLForce,2)-Mathf.Pow(maxLF,2);
                }
                float F_CosntraintRF(float force,float torque,float dammyRForce){
                    return Mathf.Pow((force/Mathf.Cos(BodyTheta[i])+torque-maxRF)/2,2)+Mathf.Pow(dammyRForce,2)-Mathf.Pow(maxRF,2);
                }
                Difference_Force[i]=-StableConstant*F_Force(Force[i],Torque[i],Lag_LForce[i],Lag_RForce[i])
                                                -(F_Force(Force[i]+DiffForce[i]*EvalDT, Torque[i]+DiffTorque[i]*EvalDT,Lag_LForce[i]+DiffLagLF[i]*EvalDT,Lag_RForce[i]+DiffLagRF[i]*EvalDT)
                                                    -F_Force(Force[i],Torque[i],Lag_LForce[i],Lag_RForce[i]))/EvalDT; 
                Difference_Torque[i]=-StableConstant*F_Torque(Force[i],Torque[i],Lag_LForce[i],Lag_RForce[i])
                                                -(F_Torque(Force[i]+DiffForce[i]*EvalDT,Torque[i]+DiffTorque[i]*EvalDT,Lag_LForce[i]+DiffLagLF[i]*EvalDT,Lag_RForce[i]+DiffLagRF[i]*EvalDT)
                                                    -F_Torque(Force[i],Torque[i],Lag_LForce[i],Lag_RForce[i]))/EvalDT;
                Difference_DammyTheta[i]=-StableConstant*F_DammyTheta(DammyTheta[i],Lag_theta[i])-(F_DammyTheta(DammyTheta[i]+DiffDammyTheta[i]*EvalDT,Lag_theta[i]+DiffLagTh[i]*EvalDT)
                                                                                                        -F_DammyTheta(DammyTheta[i],Lag_theta[i]))/EvalDT;
                Difference_DammyLForce[i]=-StableConstant*F_DammyLForce(DammyLForce[i],Lag_LForce[i])-(F_DammyLForce(DammyLForce[i]+DiffDammyLForce[i]*EvalDT,Lag_LForce[i]+DiffLagLF[i]*EvalDT)
                                                                                                        -F_DammyLForce(DammyLForce[i],Lag_LForce[i]))/EvalDT;
                Difference_DammyRForce[i]=-StableConstant*F_DammyRForce(DammyRForce[i],Lag_RForce[i])-(F_DammyRForce(DammyRForce[i]+DiffDammyRForce[i]*EvalDT,Lag_RForce[i]+DiffLagRF[i]*EvalDT)
                                                                                                        -F_DammyRForce(DammyRForce[i],Lag_RForce[i]))/EvalDT;
                Difference_ConstraintTh[i]=-StableConstant*F_CosntraintTh(DammyTheta[i])-(F_CosntraintTh(DammyTheta[i]+DiffDammyTheta[i]*EvalDT)-F_CosntraintTh(DammyTheta[i]))/EvalDT;
                Difference_ConstraintLF[i]=-StableConstant*F_CosntraintLF(Force[i],Torque[i],DammyLForce[i])
                                                -(F_CosntraintLF(Force[i]+DiffForce[i]*EvalDT,Torque[i]+DiffTorque[i]*EvalDT,DammyLForce[i]+DiffDammyLForce[i]*EvalDT)
                                                    -F_CosntraintLF(Force[i],Torque[i],DammyLForce[i]))/EvalDT;
                Difference_ConstraintRF[i]=-StableConstant*F_CosntraintRF(Force[i],Torque[i],DammyRForce[i])
                                                -(F_CosntraintRF(Force[i]+DiffForce[i]*EvalDT,Torque[i]+DiffTorque[i]*EvalDT,DammyRForce[i]+DiffDammyRForce[i]*EvalDT)
                                                    -F_CosntraintRF(Force[i],Torque[i],DammyRForce[i]))/EvalDT;
                Difference[i*8]=Difference_Force[i];
                Difference[i*8+1]=Difference_Torque[i];
                Difference[i*8+2]=Difference_DammyTheta[i];
                Difference[i*8+3]=Difference_DammyLForce[i];
                Difference[i*8+4]=Difference_DammyRForce[i];
                Difference[i*8+7]=Difference_ConstraintTh[i];
                Difference[i*8+5]=Difference_ConstraintLF[i];
                Difference[i*8+6]=Difference_ConstraintRF[i];
                DifferenceInnerProduct+=(Mathf.Pow(Difference[i*8],2)+Mathf.Pow(Difference[i*8+1],2)+Mathf.Pow(Difference[i*8+2],2)+Mathf.Pow(Difference[i*8+3],2)+Mathf.Pow(Difference[i*8+4],2)
                                                    +Mathf.Pow(Difference[i*8+5],2)+Mathf.Pow(Difference[i*8+6],2)+Mathf.Pow(Difference[i*8+7],2));//sqrt this later
            }
            DifferenceInnerProduct=Mathf.Sqrt(DifferenceInnerProduct);
            for(int i=0;i<PredictionTime*8; i++) OrthoBasis[0,i]=Difference[i]/DifferenceInnerProduct;

            float[,] h=new float[GMRES_RepeatTime+1, GMRES_RepeatTime];//gyo, retu 
            float[] y=new float[GMRES_RepeatTime]; 
            for(int i=0;i<GMRES_RepeatTime+1; i++){ 
                for(int j=0;j<GMRES_RepeatTime; j++){
                    h[i,j]=0;
                    y[j]=0;
                }
            }
            for(int i=0;i<GMRES_RepeatTime; i++){
                for(int j=0; j<PredictionTime; j++) {
                    float F_Force(float force,float torque,float Lag_LForce,float Lag_RForce) {
                        return ForceConstant_Stage*force+AdjointVector_dx[j+1]/Mass+Lag_LForce*(force/Mathf.Cos(BodyTheta[j])-torque-maxLF)/2/Mathf.Cos(BodyTheta[j])
                                                                                        +Lag_RForce*(force/Mathf.Cos(BodyTheta[j])+torque-maxRF)/2/Mathf.Cos(BodyTheta[j]);
                    }
                    float F_Torque(float force,float torque,float Lag_LForce,float Lag_RForce) {
                        return TorqueConstant_Stage*torque+TorqueConstent*AdjointVector_dtheta[j+1]
                                        -Lag_LForce*(force/Mathf.Cos(BodyTheta[j])-torque-maxLF)/2-Lag_RForce*(force/Mathf.Cos(BodyTheta[j])+torque-maxRF)/2;
                    }
                    float F_DammyTheta(float dammyTheta,float lag_theta) {
                        return -0.01f+2*lag_theta*dammyTheta;
                    }
                    float F_DammyLForce(float dammyLForce,float lag_LForce){
                        return -0.01f+2*lag_LForce*dammyLForce;
                    }
                    float F_DammyRForce(float dammyRForce,float lag_RForce){
                        return -0.01f+2*lag_RForce*dammyRForce;
                    }
                    float F_CosntraintTh(float dammyTheta){
                        return Mathf.Pow(BodyTheta[j]-TargetDirectionAngle,2)+Mathf.Pow(dammyTheta,2)-Mathf.Pow(maxTh,2);
                    }
                    float F_CosntraintLF(float force,float torque,float dammyLForce){
                        return Mathf.Pow((force/Mathf.Cos(BodyTheta[j])-torque-maxLF)/2,2)+Mathf.Pow(dammyLForce,2)-Mathf.Pow(maxLF,2);
                    }
                    float F_CosntraintRF(float force,float torque,float dammyRForce){
                        return Mathf.Pow((force/Mathf.Cos(BodyTheta[j])+torque-maxRF)/2,2)+Mathf.Pow(dammyRForce,2)-Mathf.Pow(maxRF,2);
                    }
                    OrthoBasis[i+1,j*8]=(F_Force(Force[j]+OrthoBasis[i,j*8]*EvalDT,Torque[j]+OrthoBasis[i,j*8+1]*EvalDT,Lag_LForce[i]+OrthoBasis[i,j*8+6]*EvalDT,Lag_RForce[j]+OrthoBasis[i,i*8+7]*EvalDT)
                                            -F_Force(Force[j],Torque[j],Lag_LForce[j],Lag_RForce[j]))/EvalDT; 
                    OrthoBasis[i+1,j*8+1]=(F_Torque(Force[j]+OrthoBasis[i,j*8]*EvalDT,Torque[j]+OrthoBasis[i,j*8+1]*EvalDT,Lag_LForce[j]+OrthoBasis[i,j*8+6]*EvalDT,Lag_RForce[j]+OrthoBasis[i,j*8+7]*EvalDT)
                                            -F_Torque(Force[j],Torque[j],Lag_LForce[j],Lag_RForce[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+2]=(F_DammyTheta(DammyTheta[j]+OrthoBasis[i,j*8+2]*EvalDT,Lag_theta[i]+OrthoBasis[i,j*8+5]*EvalDT)
                                                                                                        -F_DammyTheta(DammyTheta[j],Lag_theta[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+3]=(F_DammyLForce(DammyLForce[j]+OrthoBasis[i,j*8+3]*EvalDT,Lag_LForce[i]+OrthoBasis[i,j*8+6]*EvalDT)
                                                                                                        -F_DammyLForce(DammyLForce[j],Lag_LForce[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+4]=(F_DammyRForce(DammyRForce[j]+OrthoBasis[i,j*8+4]*EvalDT,Lag_RForce[j]+OrthoBasis[i,j*8+7]*EvalDT)
                                                                                                        -F_DammyRForce(DammyRForce[j],Lag_RForce[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+5]=(F_CosntraintTh(DammyTheta[j]+OrthoBasis[i,j*8+2]*EvalDT)-F_CosntraintTh(DammyTheta[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+6]=(F_CosntraintLF(Force[j]+OrthoBasis[i,j*8]*EvalDT,Torque[j]+OrthoBasis[i,j*8+1]*EvalDT,DammyLForce[j]+OrthoBasis[i,j*8+3]*EvalDT)
                                                    -F_CosntraintLF(Force[j],Torque[j],DammyLForce[j]))/EvalDT;
                    OrthoBasis[i+1,j*8+7]=(F_CosntraintRF(Force[j]+OrthoBasis[i,j*8]*EvalDT,Torque[j]+OrthoBasis[i,j*8+1]*EvalDT,DammyRForce[j]+OrthoBasis[i,j*8+4]*EvalDT)
                                                    -F_CosntraintRF(Force[j],Torque[j],DammyRForce[j]))/EvalDT;
                }
                for(int j=0; j<i+1;j++){
                    for(int k=0;k<PredictionTime*8;k++) h[j,i]+=OrthoBasis[i+1,k]*OrthoBasis[j,k]; 
                    for(int k=0;k<PredictionTime*8;k++) OrthoBasis[i+1,k]=OrthoBasis[i+1,k]-h[j,i]*OrthoBasis[j,k];
                }
                for(int j=0; j<PredictionTime*8; j++)h[i+1,i]+=Mathf.Pow(OrthoBasis[i+1,j],2); //sqrt this later
                h[i+1,i]=Mathf.Sqrt(h[i+1,i]);
                for(int j=0; j<PredictionTime*8; j++) OrthoBasis[i+1,j]=OrthoBasis[i+1,j]/h[i+1,i];
                
            }


            /*
            ここまでの計算により
                {1  {h00,h01  {y0
            |r|× 0 = h10,h11 × y1}
                0}   0 ,h21}  
            これをGives回転を用いて右辺のヘッセンベルグ行列を上三角行列にする
            */
            float[] GivensColumn_1=new float[GMRES_RepeatTime+1];
            for(int i=0;i<GMRES_RepeatTime+1; i++) GivensColumn_1[i]=1;

            for(int i=0;i<GMRES_RepeatTime; i++){
                float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i]*Mathf.Pow(10,10),2)+Mathf.Pow(h[i,i]*Mathf.Pow(10,10),2));
                float SinTheta=h[i+1,i]/r*Mathf.Pow(10,10); 
                float CosTheta=-h[i,i]/r*Mathf.Pow(10,10);

                for(int j=0; j<GMRES_RepeatTime+1; j++){ 
                    for(int k=0; k<GMRES_RepeatTime;k++){ 
                        if(j==i)h[j,k]=h[j,k]*CosTheta-h[j+1,k]*SinTheta; 
                        else if(j==i+1 && k==i)h[j,k]=0; 
                        else if(j==i+1 && k>i)h[j,k]=h[j-1,k]*SinTheta+h[j,k]*CosTheta;
                    }
                    if(j==i)GivensColumn_1[j]*=CosTheta;
                    else if(j>i)GivensColumn_1[j]*=SinTheta;
                }
            }

            /*
            calculate y from↓
                {G0  {h00,h01  {y0
            |r|× G1}=  0 ,h11 × y1}
                0 }   0 , 0 }   
            */

            for(int i=GMRES_RepeatTime-1;i>0-1;i--) {
                float DevidedValue=GivensColumn_1[i]*DifferenceInnerProduct;
                for(int j=GMRES_RepeatTime-1;j>i;j--) DevidedValue-=h[i,j]*y[j];
                y[i]=DevidedValue/h[i,i];
            }

            //calculate U by U=(previous)U+dU/dt*dt
            for(int i=0;i<PredictionTime*8;i++){
                for(int j=0;j<GMRES_RepeatTime;j++){
                    if(i%8==0)DiffForce[i/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==1)DiffTorque[(i-1)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==2)DiffDammyTheta[(i-2)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==3)DiffDammyLForce[(i-3)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==4)DiffDammyRForce[(i-4)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==5)DiffLagTh[(i-5)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==6)DiffLagLF[(i-6)/8]+=OrthoBasis[j,i]*y[j];
                    if(i%8==7)DiffLagRF[(i-7)/8]+=OrthoBasis[j,i]*y[j];
                }
                if(i%8==0)Force[i/8]+=DiffForce[i/8]*dt;
                if(i%8==1)Torque[(i-1)/8]+=DiffTorque[(i-1)/8]*dt;
                if(i%8==2)DammyTheta[(i-2)/8]+=DiffDammyTheta[(i-2)/8]*dt;
                if(i%8==3)DammyLForce[(i-3)/8]+=DiffDammyLForce[(i-3)/8]*dt;
                if(i%8==4)DammyRForce[(i-4)/8]+=DiffDammyRForce[(i-4)/8]*dt;
                if(i%8==5)DiffLagTh[(i-5)/8]+=DiffLagTh[(i-5)/8]*dt;
                if(i%8==6)DiffLagLF[(i-6)/8]+=DiffLagLF[(i-6)/8]*dt;
                if(i%8==7)DiffLagRF[(i-7)/8]+=DiffLagRF[(i-7)/8]*dt;
            }
            LeftForce=(Force[0]/Mathf.Cos(BodyTheta[0])-Torque[0])/2;
            RightForce=(Force[0]/Mathf.Cos(BodyTheta[0])+Torque[0])/2;
        }     
    }
}

