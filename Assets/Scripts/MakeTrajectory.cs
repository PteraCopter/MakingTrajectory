
using System.Collections;
using System.Collections.Generic; 
using UnityEngine;
//using DifiningObjects_MPControl;

/*
    前置き
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
    PositionRefference_x 0
    PositionRefference_y 3
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

public class MakeTrajectory : MonoBehaviour{
    public Natural natural;
    public MakeTargetPoint makeTargetPoint;
    public GameObject[] Obstacle;
    public GameObject[] ObstacleXLine;
    public GameObject[] ObstacleZLine;
    public int GMRES_RepeatTime=5;
    public int PredictionTime=100;
    public float StableConstant=5;
    public float XConstant=1,ZConstant=1,dXCosntant=1,  dZConstant=1,XConstant_Stage=1,ZConstant_Stage=1,dXConstant_Stage=1,dZConstant_Stage=1,ddXConstant=1,ddZConstant=1,
                                ObstacleRadius=0.3f,ObstacleEffectScope=0.5f,ObstacleCostConstant=10;
    public float FinalEvaluarionScope=5.0f;//[s]
    public float[] BodyPosition_x,BodyPosition_z;//x,z
    float[] BodyVelocity_x,BodyVelocity_z;
    public float[] BodyAcc_x,BodyAcc_z;
    float[] DiffBodyAcc_x,DiffBodyAcc_z;    
    float[] AdjointVector_x,AdjointVector_dx,AdjointVector_z,AdjointVector_dz;
    float PreviousBodyPosition_x,PreviousBodyPosition_z;
    float PositionReference_x,PositionReference_z;
    float InitialTime;
    Transform BodyTransform;
    float[] ObstaclePos_x,ObstaclePos_z;

    // Start is called before the first frame update
    void Start(){
        ObstaclePos_x=new float[Obstacle.Length+ObstacleXLine.Length+ObstacleZLine.Length];
        ObstaclePos_z=new float[Obstacle.Length+ObstacleXLine.Length+ObstacleZLine.Length];
        //ObstacleRadius=ObstacleTransform.localScale.x/2+natural.BodyWid;
        BodyPosition_x=new float[PredictionTime+1];
        BodyPosition_z=new float[PredictionTime+1];
        BodyVelocity_x=new float[PredictionTime+1];
        BodyVelocity_z=new float[PredictionTime+1];
        BodyAcc_x=new float[PredictionTime+1];
        BodyAcc_z=new float[PredictionTime+1];
        DiffBodyAcc_x=new float[PredictionTime+1]; 
        DiffBodyAcc_z=new float[PredictionTime+1];
        AdjointVector_x=new float[PredictionTime+1]; 
        AdjointVector_dx=new float[PredictionTime+1]; 
        AdjointVector_z=new float[PredictionTime+1]; 
        AdjointVector_dz=new float[PredictionTime+1]; 
        BodyTransform=natural.BodyTransform;

        for(int i = 0; i <PredictionTime+1 ; i++){
            DiffBodyAcc_x[i]=0.01f; 
            DiffBodyAcc_z[i]=0.01f; 
            BodyAcc_x[i]=0;
            BodyAcc_z[i]=0;
            AdjointVector_x[i]=0; 
            AdjointVector_dx[i]=0; 
            AdjointVector_z[i]=0; 
            AdjointVector_dz[i]=0; 
            BodyPosition_x[i]=0; 
            BodyPosition_z[i]=0; 
            BodyVelocity_x[i]=0;
            BodyVelocity_z[i]=0;
        }  
        BodyPosition_x[0]=BodyTransform.position.x; 
        BodyPosition_z[0]=BodyTransform.position.z; 
        BodyVelocity_x[0]=0;
        BodyVelocity_z[0]=0;
        BodyAcc_x[0]=0;
        BodyAcc_z[0]=0;
        DiffBodyAcc_x[0]=0.01f;
        DiffBodyAcc_z[0]=0.01f;
        PreviousBodyPosition_x=BodyPosition_x[0];  
        PreviousBodyPosition_z=BodyPosition_z[0]; 
        InitialTime=Time.time;
    }

    

    // Update is called once per frame
    void FixedUpdate(){
        if(Input.GetKeyDown(KeyCode.M)){
            PositionReference_x=makeTargetPoint.TargetPoint.x;
            PositionReference_z=makeTargetPoint.TargetPoint.y;
            BodyPosition_x[0]=BodyTransform.position.x; 
            BodyPosition_z[0]=BodyTransform.position.z; 
            BodyVelocity_x[0]=0; 
            BodyVelocity_z[0]=0;
            for(int i=0;i<ObstaclePos_x.Length;i++){
                if(i<Obstacle.Length){
                    ObstaclePos_x[i]=Obstacle[i].transform.position.x;
                    ObstaclePos_z[i]=Obstacle[i].transform.position.z;
                }else if(i<Obstacle.Length+ObstacleXLine.Length){
                    ObstaclePos_x[i]=BodyPosition_x[0];
                    ObstaclePos_z[i]=ObstacleXLine[i-Obstacle.Length].transform.position.z;
                }else if(i<Obstacle.Length+ObstacleXLine.Length+ObstacleZLine.Length){
                    ObstaclePos_x[i]=ObstacleZLine[i-Obstacle.Length-ObstacleXLine.Length].transform.position.x;
                    ObstaclePos_z[i]=BodyPosition_x[0];
                }
            }


            for(int loopTime=0;loopTime<20;loopTime++){
                // モデルへの入力はnaturalクラスでおこなっている

                //difine EvaluationTime in this loop
                //EvalutionTime will converge to FinalEvaluationScope/PredictionTime
                //予測範囲を予測範囲分割数で割った予測空間内でのループ周期(dτ)を設定する
                //予め設定した最終予測範囲に収束するように設定する：ループ開始時は予測の精度が高くないため予測範囲は初めは0で0→最終予測範囲となるように
                float EvalDT=FinalEvaluarionScope*(1-Mathf.Exp(-10f*(Time.time-InitialTime+0.01f)))/PredictionTime; 

                //forsee ObjectPosition[i] and ObjectVelocity[i] using ObjectPosition[0] and ObjectVelocity[0], given InputPosition[i]
                //上で与えられたv[τ=0],x[τ=0](つまりv[t],x[t])とuからx[i],v[i]を順に予想していく
                for(int i=1;i<PredictionTime+1; i++) {
                    BodyVelocity_x[i]=BodyVelocity_x[i-1]+BodyAcc_x[i-1]*EvalDT;
                    BodyVelocity_z[i]=BodyVelocity_z[i-1]+BodyAcc_z[i-1]*EvalDT;
                    BodyPosition_x[i]=BodyPosition_x[i-1]+BodyVelocity_x[i-1]*EvalDT;//+force_x/Mass*EvalDT*EvalDT/2; 
                    BodyPosition_z[i]=BodyPosition_z[i-1]+BodyVelocity_z[i-1]*EvalDT;//+force_y/Mass*EvalDT*EvalDT/2; 
                    for(int j=0;j<ObstaclePos_x.Length;j++){
                        if(j<Obstacle.Length){
                            ObstaclePos_x[j]=Obstacle[j].transform.position.x;
                            ObstaclePos_z[j]=Obstacle[j].transform.position.z;
                        }else if(j<Obstacle.Length+ObstacleXLine.Length){
                            ObstaclePos_x[j]=BodyPosition_x[i];
                            ObstaclePos_z[j]=ObstacleXLine[j-Obstacle.Length].transform.position.z;
                        }else if(j<Obstacle.Length+ObstacleXLine.Length+ObstacleZLine.Length){
                            ObstaclePos_x[j]=ObstacleZLine[j-Obstacle.Length-ObstacleXLine.Length].transform.position.x;
                            ObstaclePos_z[j]=BodyPosition_x[i];
                        }
                    }
                }


                //calculate AdjointVector[i,0]and[i,1] :[i,0] for position, [i,1] for velocity
                //随伴変数を求める
                //at first, AdjointVector[last] is calculated by AdjointVector[last]=ð(TerminalCost)/ðx
                //初めに、予測範囲内で最終の随伴変数を求める、これは終端コストのx[N*dτ]での偏微分に等しい
                AdjointVector_x[PredictionTime]=XConstant*(BodyPosition_x[PredictionTime] -PositionReference_x); 
                AdjointVector_dx[PredictionTime]=dXCosntant*BodyVelocity_x[PredictionTime];
                AdjointVector_z[PredictionTime]=ZConstant*(BodyPosition_z[PredictionTime] -PositionReference_z); 
                AdjointVector_dz[PredictionTime]=dZConstant*BodyVelocity_z[PredictionTime];

                //following AdjointVector[last], AdjointVector[last -1] can be calculated by AdjointVector[last -1]=AdjointVector[last]+ ðH/ðx*dτ, and so on.
                //逆順に随伴変数を求めていく。AdjointVector[i-1]=AdjointVector[i]+ ðH/ðx*dτのように求められる。
                for(int i=PredictionTime-1;i>0;i--){ 
                    float[] ObstacleDistance=new float[ObstaclePos_x.Length];
                    float ObstacleCost_x=0,ObstacleCost_z=0;
                    for(int j=0;j<ObstaclePos_x.Length;j++){
                        ObstacleDistance[j]=new Vector2(ObstaclePos_x[j]-BodyPosition_x[i],ObstaclePos_z[j]-BodyPosition_z[i]).magnitude;
                        //Debug.Log(j+":x:"+ObstaclePos_x[j]+":y:"+ObstaclePos_z[j]);
                        if(ObstacleDistance[j]>ObstacleRadius+ObstacleEffectScope){
                            ObstacleCost_x+=0;
                            ObstacleCost_z+=0;
                        }else if(ObstacleDistance[j]>ObstacleRadius){
                            ObstacleCost_x+=2*ObstacleCostConstant/ObstacleEffectScope*(ObstaclePos_x[j]-BodyPosition_x[i])/ObstacleDistance[j]
                                                *(ObstacleDistance[j]-ObstacleRadius-ObstacleEffectScope)
                                                /Mathf.Sqrt(Mathf.Pow(ObstacleEffectScope,2)-Mathf.Pow(ObstacleDistance[j]-ObstacleRadius-ObstacleEffectScope,2));
                            ObstacleCost_z+=2*ObstacleCostConstant/ObstacleEffectScope*(ObstaclePos_z[j]-BodyPosition_z[i])/ObstacleDistance[j]
                                                *(ObstacleDistance[j]-ObstacleRadius-ObstacleEffectScope)
                                                /Mathf.Sqrt(Mathf.Pow(ObstacleEffectScope,2)-Mathf.Pow(ObstacleDistance[j]-ObstacleRadius-ObstacleEffectScope,2));
                        }else {
                            ObstacleCost_x+=2*ObstacleCostConstant*(ObstaclePos_x[j]-BodyPosition_x[i])/Mathf.Sqrt(ObstacleRadius*ObstacleRadius-ObstacleDistance[j]*ObstacleDistance[j]);
                            ObstacleCost_z+=2*ObstacleCostConstant*(ObstaclePos_z[j]-BodyPosition_z[i])/Mathf.Sqrt(ObstacleRadius*ObstacleRadius-ObstacleDistance[j]*ObstacleDistance[j]);
                        }
                    }
                    Debug.Log(ObstacleCost_x+":"+ObstacleCost_z);
                    float AdXContent=XConstant_Stage*(BodyPosition_x[i]-PositionReference_x)+ObstacleCost_x;
                    float AdZContent=ZConstant_Stage*(BodyPosition_z[i]-PositionReference_z)+ObstacleCost_z;
                    float AdDXContent=dXConstant_Stage*BodyVelocity_x[i]+AdjointVector_x[i+1];
                    float AdDZContent=dZConstant_Stage*BodyVelocity_z[i]+AdjointVector_z[i+1];
                                            
                    AdjointVector_x[i]=AdjointVector_x[i+1]+AdXContent*EvalDT;
                    AdjointVector_dx[i]=AdjointVector_dx[i+1]+AdDXContent*EvalDT;
                    AdjointVector_z[i]=AdjointVector_z[i+1]+AdZContent*EvalDT;
                    AdjointVector_dz[i]=AdjointVector_dz[i+1]+AdDZContent*EvalDT;
                }


                //calculate dU/dt using GMRES method
                float[] Difference_AccX=new float[PredictionTime];
                float[] Difference_AccZ=new float[PredictionTime];
                float[] Difference=new float[PredictionTime*2];
                float DifferenceInnerProduct=0; 
                float[,] OrthogonalBasis=new float[GMRES_RepeatTime+1, PredictionTime*2]; 

                for(int i=0;i<PredictionTime;i++) {
                    float F_AccX(float bodyAcc_x){
                        return ddXConstant*bodyAcc_x+AdjointVector_dx[i+1];
                    }
                    float F_AccZ(float bodyAcc_z){
                        return +ddZConstant*bodyAcc_z+AdjointVector_dz[i+1];
                    }
                    Difference_AccX[i]=-StableConstant*F_AccX(BodyAcc_x[i])
                                        -(F_AccX(BodyAcc_x[i]+DiffBodyAcc_x[i]*EvalDT)-F_AccX(BodyAcc_x[i]))/EvalDT; 
                    Difference_AccZ[i]=-StableConstant*F_AccZ(BodyAcc_z[i])
                                        -(F_AccZ(BodyAcc_z[i]+DiffBodyAcc_z[i]*EvalDT)-F_AccZ(BodyAcc_z[i]))/EvalDT; 
                    Difference[i*2]=Difference_AccX[i];
                    Difference[i*2+1]=Difference_AccZ[i];
                    DifferenceInnerProduct+=(Mathf.Pow(Difference_AccX[i], 2)+Mathf.Pow(Difference_AccZ[i],2));//sqrt this later
                }
                DifferenceInnerProduct=Mathf.Sqrt(DifferenceInnerProduct);
                for(int i=0;i<PredictionTime*2; i++) OrthogonalBasis[0,i]=Difference[i]/DifferenceInnerProduct;

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
                        float F_AccX(float bodyAcc_x){
                            return ddXConstant*bodyAcc_x+AdjointVector_dx[j+1];
                        }
                        float F_AccZ(float bodyAcc_z){
                            return ddZConstant*bodyAcc_z+AdjointVector_dz[j+1];
                        }

                        OrthogonalBasis[i+1,j*2]=(F_AccX(BodyAcc_x[j]+OrthogonalBasis[i,j*2]*EvalDT)-F_AccX(BodyAcc_x[j]))/EvalDT; 
                        OrthogonalBasis[i+1,j*2+1]=(F_AccZ(BodyAcc_z[j]+OrthogonalBasis[i,j*2+1]*EvalDT)-F_AccZ(BodyAcc_z[j]))/EvalDT; 
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
                    {1  {h00,h01  {y0
                |r|× 0 = h10,h11 × y1}
                    0}   0 ,h21}  
                これをGives回転を用いて右辺のヘッセンベルグ行列を上三角行列にする
                */
                float[] GivensColumn_1=new float[GMRES_RepeatTime+1];
                for(int i=0;i<GMRES_RepeatTime+1; i++) GivensColumn_1[i]=1;

                for(int i=0;i<GMRES_RepeatTime; i++){
                    float r=Mathf.Sqrt(Mathf.Pow(h[i+1,i],2)+Mathf.Pow(h[i,i],2));
                    float SinTheta=h[i+1,i]/r; 
                    float CosTheta=-h[i,i]/r;

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
                for(int i=0;i<PredictionTime*2;i++){
                    for(int j=0;j<GMRES_RepeatTime;j++){
                        if(i%2==0)DiffBodyAcc_x[i/2]+=OrthogonalBasis[j,i]*y[j];
                        if(i%2==1)DiffBodyAcc_z[(i-1)/2]+=OrthogonalBasis[j,i]*y[j];
                    }
                    if(i%2==0)BodyAcc_x[i/2]+=DiffBodyAcc_x[i/2]*EvalDT;
                    if(i%2==1)BodyAcc_z[(i-1)/2]+=DiffBodyAcc_z[(i-1)/2]*EvalDT;
                }
            }
        }
    }
}

