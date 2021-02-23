using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MakeTargetPoint : MonoBehaviour
{
    public bool AutoMakePoint=false;
    public float x_limit=7;
    public float z_limit=7;
    public Vector2 TargetPoint=new Vector2(2,2);
    public Transform BodyTransform;
    public Transform TargetPointIndicater;
    int AchieveTime=0;
    public MakeTrajectory makeTrajectory;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        bool FullfillTarget=false;
        bool CloseToObstacle=false;
        Vector2 CurPosition=new Vector2(BodyTransform.position.x,BodyTransform.position.z);
        if(Vector2.Distance(CurPosition,TargetPoint)<0.7f) FullfillTarget=true;
        if(AutoMakePoint){
            for(int i=0;i<makeTrajectory.Obstacle.Length;i++){
                Vector2 ObstacleVector=new Vector2(makeTrajectory.Obstacle[i].transform.position.x,makeTrajectory.Obstacle[i].transform.position.z);
                if(Vector2.Distance(ObstacleVector,TargetPoint)<1.3f) CloseToObstacle=true;
            }
            if(FullfillTarget||CloseToObstacle){
                AchieveTime++;
                float Target_x=Random.Range(-x_limit,x_limit);
                float Target_z=Random.Range(z_limit*(-1),z_limit*1);
                TargetPoint=new Vector2(Target_x,Target_z);
            }
            TargetPointIndicater.position=new Vector3(TargetPoint.x,-0.5f,TargetPoint.y);
        }else{
            TargetPoint=new Vector2(TargetPointIndicater.position.x,TargetPointIndicater.position.z);
        }
    }
}