
using System.Collections;
using System.Collections.Generic; 
using UnityEngine;
//using DifiningObjects_MPControl;

/*
    このプログラムに対して評価関数からばねが自然長から離れる距離に比例するコストを抜き入力の時間微分のコストを追加したものである。

    前置き
    コメントアウト機能を利用して各式の説明を行っているが、その時に利用する文字等について先に示しておく
    t:そのループ時の時刻
    τ:予測範囲での時刻、時刻tのときτ=0である
    d~:微小な~ ex:dx/dtはxの時間微分表す
    ð~:ex:ðx/ðuはxのuによる偏微分を表す
    x[t]:時刻tにおける目標物の位置
    v[t]:時刻tにおける目標物の速度
    u[t]:時刻tにおける入力

    パラメータ記録
*/

public class MPControl : MonoBehaviour{
    public bool MPC_mode=true;
    public float PositionReference_x=0;
    public float PositionReference_y=0;
    public Natural natural;
    public int GMRES_RepeatTime=2;
    public int PredictionTime=10;
    public float StableConstant=100;
    GameObject Body;
    public float XConstant,XConstant_Stage,dXConstant_Stage, YConstant,YConstant_Stage,dYConstant_Stage,
                                ThetaConstant,ThetaConstant_Stage,dThetaConstant_Stage, LForceConstant_Stage,RForceConstant_Stage;
    public float FinalEvaluarionScope=0.5f;//[s]
    public float[] BodyPosition_x,BodyPosition_y;//x,y
    public float[] BodyAngle;
    Transform BodyTransform;
    Rigidbody2D rb;
    float[] BodyVelocity_x,BodyVelocity_y;
    float[] BodyRev;//=d(BodyAngle)/dt
    public float[] LeftPower,RightPower;
    float[] DifferentialLeftPower,DifferentialRightPower;    
    float[] AdjointVector_x,AdjointVector_dx,AdjointVector_y,AdjointVector_dy,AdjointVector_theta,AdjointVector_dtheta;
    float PreviousBodyPosition_x,PreviousBodyPosition_y,PreviousBodyAngle;
    float TorqueConstent;
    float InitialTime;
    float GrabityConstant;
    float Mass;
    const float DegToRad=Mathf.PI/180
    
    
    
    ;

    // Start is called before the first frame update
    void Start(){
        BodyPosition_x=new float[PredictionTime+1];
        BodyPosition_y=new float[PredictionTime+1];
        BodyVelocity_x=new float[PredictionTime+1];
        BodyVelocity_y=new float[PredictionTime+1];
        BodyAngle=new float[PredictionTime+1];
        BodyRev=new float[PredictionTime+1];
        LeftPower=new float[PredictionTime+1];
        RightPower=new float[PredictionTime+1];
        DifferentialLeftPower=new float[PredictionTime+1]; 
        DifferentialRightPower=new float[PredictionTime+1];
        AdjointVector_x=new float[PredictionTime+1]; 
        AdjointVector_dx=new float[PredictionTime+1]; 
        AdjointVector_y=new float[PredictionTime+1]; 
        AdjointVector_dy=new float[PredictionTime+1]; 
        AdjointVector_theta=new float[PredictionTime+1]; 
        AdjointVector_dtheta=new float[PredictionTime+1];
        Body=natural.Body;
        rb=Body.GetComponent<Rigidbody2D>();
        BodyTransform=Body.GetComponent<Transform>();
        TorqueConstent=natural.TorqueConstant;
        Mass=rb.mass;
        GrabityConstant=natural.GrabityConstant;

        for(int i = 0; i <PredictionTime+1 ; i++){
            DifferentialLeftPower[i]=0; 
            DifferentialRightPower[i]=0; 
            LeftPower[i]=0;
            RightPower[i]=0;
            AdjointVector_x[i]=0; 
            AdjointVector_dx[i]=0; 
            AdjointVector_y[i]=0; 
            AdjointVector_dy[i]=0; 
            AdjointVector_theta[i]=0; 
            AdjointVector_dtheta[i]=0; 
            BodyPosition_x[i]=0; 
            BodyPosition_y[i]=0; 
            BodyVelocity_x[i]=0;
            BodyVelocity_y[i]=0;
            BodyAngle[i]=0;
            BodyRev[i]=0;
        }  
        BodyPosition_x[0]=BodyTransform.position.x; 
        BodyPosition_y[0]=BodyTransform.position.y; 
        BodyVelocity_x[0]=0;
        BodyVelocity_y[0]=0;
        BodyAngle[0]=0;
        BodyRev[0]=0; 
        LeftPower[0]=Mass*GrabityConstant/2; 
        RightPower[0]=Mass*GrabityConstant/2;
        for(int i=1;i<PredictionTime;i++){
            LeftPower[i]=LeftPower[0];
            RightPower[i]=RightPower[0];
        }
        PreviousBodyPosition_x=BodyPosition_x[0];  
        PreviousBodyPosition_y=BodyPosition_y[0]; 
        PreviousBodyAngle=BodyAngle[0];
        InitialTime=Time.time;
    }

    

    // Update is called once per frame
    void FixedUpdate(){
        
        Debug.Log(LeftPower[0]);
        // モデルへの入力
        if(MPC_mode){
            rb.AddForce(new Vector2( -(LeftPower[0]+RightPower[0])*Mathf.Sin(BodyAngle[0]*DegToRad)/Mass,(LeftPower[0]+RightPower[0])*Mathf.Cos(BodyAngle[0]*DegToRad)/Mass));
            rb.AddTorque(-TorqueConstent*(LeftPower[0]-RightPower[0]));
        }

        //measure real delta time (not evaluation delta time)
        //制御ループ周期(dt)測定
        float dt=Time.deltaTime; 

        //meature present Object's position and velocity and input them into ObjectPosition[0],ObjectVelocity[0]
        //目標物の位置と速度を計測し、x[τ=0],v[τ=0]に代入する
        BodyPosition_x[0]=BodyTransform.position.x; 
        BodyPosition_y[0]=BodyTransform.position.y; 
        BodyVelocity_x[0]=(BodyPosition_x[0]-PreviousBodyPosition_x)/dt; 
        BodyVelocity_y[0]=(BodyPosition_y[0]-PreviousBodyPosition_y)/dt; 
        BodyAngle[0]=BodyTransform.localEulerAngles.z;
        BodyRev[0]=(BodyAngle[0]-PreviousBodyAngle)/dt;
        PreviousBodyPosition_x=BodyPosition_x[0]; 
        PreviousBodyPosition_y=BodyPosition_y[0]; 
        PreviousBodyAngle=BodyAngle[0];

        //difine EvaluationTime in this loop
        //EvalutionTime will converge to FinalEvaluationScope/PredictionTime
        //予測範囲を予測範囲分割数で割った予測空間内でのループ周期(dτ)を設定する
        //予め設定した最終予測範囲に収束するように設定する：ループ開始時は予測の精度が高くないため予測範囲は初めは0で0→最終予測範囲となるように
        float EvaluationDeltaTime=FinalEvaluarionScope*(1-Mathf.Exp(-2.5f*(Time.time-InitialTime+0.01f)))/PredictionTime; 

        //forsee ObjectPosition[i] and ObjectVelocity[i] using ObjectPosition[0] and ObjectVelocity[0], given InputPosition[i]
        //上で与えられたv[τ=0],x[τ=0](つまりv[t],x[t])とuからx[i],v[i]を順に予想していく
        for(int i=1;i<PredictionTime+1; i++) {
            float force_x=(LeftPower[i-1]+RightPower[i-1])*Mathf.Sin(BodyAngle[i-1]*DegToRad);
            float force_y=(LeftPower[i-1]+RightPower[i-1])*Mathf.Cos(BodyAngle[i-1]*DegToRad)-Mass*GrabityConstant;
            BodyPosition_x[i]=BodyPosition_x[i-1]+BodyVelocity_x[i-1]*EvaluationDeltaTime+force_x*EvaluationDeltaTime*EvaluationDeltaTime/2; 
            BodyPosition_y[i]=BodyPosition_y[i-1]+BodyVelocity_y[i-1]*EvaluationDeltaTime+force_y*EvaluationDeltaTime*EvaluationDeltaTime/2; 
            BodyVelocity_x[i]=BodyVelocity_x[i-1]+force_x*EvaluationDeltaTime;
            BodyVelocity_y[i]=BodyVelocity_y[i-1]+force_y*EvaluationDeltaTime;
            BodyAngle[i]=BodyAngle[i-1]+BodyRev[i-1]*EvaluationDeltaTime+TorqueConstent*(LeftPower[i-1]-RightPower[i-1])*EvaluationDeltaTime*EvaluationDeltaTime/2;
            BodyRev[i]=BodyRev[i-1]+TorqueConstent*(LeftPower[i-1]-RightPower[i-1])*EvaluationDeltaTime;
        }

        //calculate AdjointVector[i,0]and[i,1] :[i,0] for position, [i,1] for velocity
        //随伴変数を求める
        //at first, AdjointVector[last] is calculated by AdjointVector[last]=ð(TerminalCost)/ðx
        //初めに、予測範囲内で最終の随伴変数を求める、これは終端コストのx[N*dτ]での偏微分に等しい
        AdjointVector_x[PredictionTime]=XConstant*(BodyPosition_x[PredictionTime] -PositionReference_x); 
        AdjointVector_dx[PredictionTime]=0;
        AdjointVector_y[PredictionTime]=YConstant*(BodyPosition_y[PredictionTime] -PositionReference_y); 
        AdjointVector_dy[PredictionTime]=0;
        AdjointVector_theta[PredictionTime]=ThetaConstant*BodyAngle[PredictionTime]; 
        AdjointVector_dtheta[PredictionTime]=0;

        //following AdjointVector[last], AdjointVector[last -1] can be calculated by AdjointVector[last -1]=AdjointVector[last]+ ðH/ðx, and so on.
        //
        for(int i=PredictionTime-1;i>0;i--){ 
            AdjointVector_x[i]=AdjointVector_x[i+1]+XConstant_Stage*(BodyPosition_x[i]-PositionReference_x)*EvaluationDeltaTime;
            AdjointVector_dx[i]=AdjointVector_dx[i+1]+(dXConstant_Stage*BodyVelocity_x[i]+AdjointVector_x[i+1])*EvaluationDeltaTime;
            AdjointVector_y[i]=AdjointVector_y[i+1]+YConstant_Stage*(BodyPosition_y[i]-PositionReference_y)*EvaluationDeltaTime;
            AdjointVector_dy[i]=AdjointVector_dy[i+1]+(dYConstant_Stage*BodyVelocity_y[i]+AdjointVector_y[i+1])*EvaluationDeltaTime;
            AdjointVector_theta[i]=AdjointVector_theta[i+1]+(ThetaConstant_Stage*BodyAngle[i]-AdjointVector_dx[i+1]/Mass*(LeftPower[i]+RightPower[i])*Mathf.Cos(BodyAngle[i]*DegToRad)
                                                            -AdjointVector_dy[i+1]/Mass*(LeftPower[i]+RightPower[i])*Mathf.Sin(BodyAngle[i]*DegToRad))*EvaluationDeltaTime;
            AdjointVector_dtheta[i]=AdjointVector_dtheta[i+1]+(dThetaConstant_Stage*BodyRev[i]+AdjointVector_theta[i+1])*EvaluationDeltaTime;
        }

        //calculate dU/dt using GMRES method
        float[] Difference_Left=new float[PredictionTime];
        float[] Difference_Rigit=new float[PredictionTime];
        float[] Difference=new float[PredictionTime*2];
        float DifferenceInnerProduct=0; 
        float[,] OrthogonalBasis=new float[GMRES_RepeatTime+1, PredictionTime*2]; 

        for(int i=0;i<PredictionTime;i++) {
            float F_LeftPower=LForceConstant_Stage*LeftPower[i]-AdjointVector_dx[i+1]/Mass*Mathf.Sin(BodyAngle[i]*DegToRad)
                                    +AdjointVector_dy[i+1]/Mass*Mathf.Cos(BodyAngle[i]*DegToRad)-TorqueConstent*AdjointVector_dtheta[i+1];
            float F_RightPower=RForceConstant_Stage*RightPower[i]-AdjointVector_dx[i+1]/Mass*Mathf.Sin(BodyAngle[i]*DegToRad)
                                    +AdjointVector_dy[i+1]/Mass*Mathf.Cos(BodyAngle[i]*DegToRad)+TorqueConstent*AdjointVector_dtheta[i+1];
            float DhF_LeftPower=LForceConstant_Stage*(LeftPower[i]+Difference_Left[i]*EvaluationDeltaTime)-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                    +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)-TorqueConstent*AdjointVector_dtheta[i+1];
            float DhF_RightPower=RForceConstant_Stage*(RightPower[i]+Difference_Rigit[i]*EvaluationDeltaTime)-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                    +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)+TorqueConstent*AdjointVector_dtheta[i+1];
            Difference_Left[i]=-StableConstant*F_LeftPower-(DhF_LeftPower-F_LeftPower)/EvaluationDeltaTime; 
            Difference_Rigit[i]=-StableConstant*F_RightPower-(DhF_RightPower-F_RightPower)/EvaluationDeltaTime;
            Difference[i*2]=Difference_Left[i];
            Difference[i*2+1]=Difference_Rigit[i];
            DifferenceInnerProduct+=Mathf.Pow(Difference_Left[i], 2)+Mathf.Pow(Difference_Rigit[i],2);//sqrt this later
        }
        DifferenceInnerProduct=Mathf.Sqrt(DifferenceInnerProduct);

        for(int i=0;i<PredictionTime; i++) OrthogonalBasis[0,i]=Difference[i]/DifferenceInnerProduct;

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
                float F_LeftPower=LForceConstant_Stage*LeftPower[i]-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                        +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)-TorqueConstent*AdjointVector_dtheta[i+1];
                float F_RightPower=RForceConstant_Stage*RightPower[i]-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                        +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)+TorqueConstent*AdjointVector_dtheta[i+1];
                float DhF_LeftPower_v=LForceConstant_Stage*(LeftPower[i]+OrthogonalBasis[i,j]*EvaluationDeltaTime)-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                        +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)-TorqueConstent*AdjointVector_dtheta[i+1];
                float DhF_RightPower_v=RForceConstant_Stage*(RightPower[i]+OrthogonalBasis[i,j]*EvaluationDeltaTime)-AdjointVector_dx[i+1]/Mass*Mathf.Sin((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)
                                        +AdjointVector_dy[i+1]/Mass*Mathf.Cos((BodyAngle[i]+BodyRev[i]*EvaluationDeltaTime)*DegToRad)+TorqueConstent*AdjointVector_dtheta[i+1];
                OrthogonalBasis[i+1,j*2]=(DhF_LeftPower_v-F_LeftPower)/EvaluationDeltaTime; 
                OrthogonalBasis[i+1,j*2+1]=(DhF_RightPower_v-F_RightPower)/EvaluationDeltaTime; 
            }
            for(int j=0; j<i+1;j++){
                for(int k=0;k<PredictionTime*2;k++) h[j,i]+=OrthogonalBasis[i+1,k]*OrthogonalBasis[j,k]; 
                for(int k=0;k<PredictionTime*2;k++) OrthogonalBasis[i+1,k]=OrthogonalBasis[i+1,k]-h[j,i]*OrthogonalBasis[j,k];
            }
            for(int j=0; j<PredictionTime*2; j++)h[i+1,i]+=Mathf.Pow(OrthogonalBasis[i+1,j],2); //sqrt this later
            h[i+1,i]=Mathf.Sqrt(h[i+1,i]);
            for(int j=0; j<PredictionTime*2; j++) OrthogonalBasis[i+1,j]=OrthogonalBasis[i+1,j]/h[i+1,i];
            
        }


        /*
        ここまでの計算により
        {       {
              +
            }       }
        */
        float[] GivensColumn_1=new float[GMRES_RepeatTime+1];
        for(int i=0;i<GMRES_RepeatTime+1; i++) GivensColumn_1[i]=1;

        for(int i=0;i<GMRES_RepeatTime; i++){
            float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i],2)+Mathf.Pow(h[i,i],2));//sqrt((h*10^(-10))^2+(^2) 
            float SinTheta=h[i+1,i]/r; 
            float CosTheta=-h[i,i]/r;

            for(int j=0; j<GMRES_RepeatTime+1; j++){ 
                for(int k=0; k<GMRES_RepeatTime;k++){ 
                    if(j==i)h[j,k]=h[j,k]*CosTheta-h[j+1,k]*SinTheta; 
                    else if(j==i+1 && k==i)h[j,k]=0; 
                    else if(j==i+1 && k>i)h[j,k]=h[j-1,k]*SinTheta+h[j,k]*CosTheta;
                    /*float TimesValue=1; 
                    if(k==i){
                        if(j==i+1) TimesValue=SinTheta; 
                        else if(j<i+1)TimesValue=CosTheta;
                        else if(j>i+1)TimesValue=0; 
                    }else if(k==i+1){
                        if(j==i+1)TimesValue=CosTheta; 
                        else if(j<i+1)TimesValue=-SinTheta; 
                        else TimesValue=1;
                    }
                    GivensColumn_1[j,k]*=TimesValue; //this would be a inefficient way*/
                }
                if(j==i)GivensColumn_1[j]*=CosTheta;
                else if(j>i)GivensColumn_1[j]*=SinTheta;
            }
        }

        for(int i=GMRES_RepeatTime-1;i>0-1;i--) {
            float DevidedValue=GivensColumn_1[i]*DifferenceInnerProduct;
            for(int j=GMRES_RepeatTime-1;j>i;j--) DevidedValue-=h[i,j]*y[j];
            y[i]=DevidedValue/h[i,i];
        }

        //calculate U by U=(previous)U+dU/dt*dt
        for(int i=0;i<PredictionTime*2;i++){
            for(int j=0;j<GMRES_RepeatTime;j++){
                if(i%2==0)DifferentialLeftPower[i/2]+=OrthogonalBasis[j,i]*y[j];
                if(i%2==1)DifferentialRightPower[(i-1)/2]+=OrthogonalBasis[j,i]*y[j];
            }
            LeftPower[i/2]+=DifferentialLeftPower[i/2]*dt;
            RightPower[(i-1)/2]+=DifferentialLeftPower[(i-1)/2]*dt;
        }

        
    }
}

