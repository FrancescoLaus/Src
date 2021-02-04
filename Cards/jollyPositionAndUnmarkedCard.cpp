#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/BHMath.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include <iostream>

using namespace std;

#define JOLLY_MOVE_THRESHOLD 250
#define JOLLY_WALK_THRESHOLD 900


CARD(jollyPositionAndUnmarkedCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkToTargetPathPlanner),
  CALLS(KeyFrameArms),


  REQUIRES(FieldBall),
  REQUIRES(ObstacleModel),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallModel),
  REQUIRES(GlobalFieldCoverage),
  REQUIRES(TeamBallModel),
  REQUIRES(GameInfo),
  REQUIRES(LibCheck),
  REQUIRES(RobotInfo),

  USES(TeamData),


  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (float)(1000.f) radiusDef,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    }),
});

class jollyPositionAndUnmarkedCard : public jollyPositionAndUnmarkedCardBase
{
  
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
      return true;
  }

  option
  {

    initial_state(stand)
    {
      transition
      {
          Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
            {
                goto receivePass;
            }
            if(theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 400) && theRobotPose.translation.x() > theLibCheck.getJollyPosition().x() -100) {
                goto smarc;
            }
            if((theRobotPose.translation-theLibCheck.getJollyPosition()).norm() > JOLLY_MOVE_THRESHOLD)
            {
                goto movePlan;
            }
      }
      action
      {      
        theStandSkill();
        theLookLeftAndRightSkill();
      }
    }

    state(movePlan)
    {
        transition
        {    
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                {
                    goto receivePass;
                }
            if((theRobotPose.translation - theLibCheck.getJollyPosition()).norm() < 50) {
                goto receivePass;
            }
            if(theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 400)) {
                if(theRobotPose.translation.x() > 2500) {
                    goto moveWalk;
            }
            if(theRobotPose.translation.x() > theLibCheck.getJollyPosition().x() -100)   {
                goto smarc;
                }           
            }
        }
        action
        {
            if(!theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 400)) {
                theKeyFrameArmsSkill(ArmKeyFrameRequest::back,true);
            }
            theLookLeftAndRightSkill();
            theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
        }
    }

    state(walkToJollyPosition) 
    {
        transition
        {
            if(theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 400)) {
                goto smarc;
            }
        }
        action
        {
            theLookLeftAndRightSkill();
            theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
        }
    }
    
    state(smarc) {
        transition {
       
            if(!theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 400)) {
                goto receivePass; 
            }
            if(theRobotPose.translation.x() > 4000) {
                    goto moveWalk;
            }
        }
        action {
        theLookForwardSkill();
            std::vector<Pose2f> freeAreas;
            for(const auto& cel : theGlobalFieldCoverage.grid) {    
                if(!theLibCheck.opponentOnArea(cel.positionOnField.x(), cel.positionOnField.y())){
                    if(theFieldBall.positionOnField.y() < 0) {
                        if(cel.positionOnField.x() > theRobotPose.translation.x()+1000 && cel.positionOnField.x() < theFieldDimensions.xPosOpponentGroundline && cel.positionOnField.y() < 0 && cel.positionOnField.y() > theRobotPose.translation.y()-500 && cel.positionOnField.y() < theRobotPose.translation.y()+500){
                            freeAreas.push_back(Pose2f(cel.positionOnField.x(), cel.positionOnField.y()));
                        } else {
                            if(cel.positionOnField.x() > theRobotPose.translation.x()+500 && cel.positionOnField.x() < theFieldDimensions.xPosOpponentGroundline && cel.positionOnField.y() > 0 && cel.positionOnField.y() > theRobotPose.translation.y()-500 && cel.positionOnField.y() < theRobotPose.translation.y()+500) {
                            freeAreas.push_back(Pose2f(cel.positionOnField.x(), cel.positionOnField.y()));
                        }
                        }
                    }
                }
                if(freeAreas.size() > 0) {
                    break;
                }              
            }     
            if(freeAreas.size() > 0){
                float x,y;
                x=freeAreas.at(0).translation.x();
                y=freeAreas.at(0).translation.y();
                theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),Pose2f(x, y));
            } else {
                theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
                }
           }
    }

    state(receivePass)
    {
        transition
        {
            if(theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 750)) {
                if(theRobotPose.translation.x() > 3000) {
                    goto moveWalk;
                } else {
                    goto smarc;
                }
            }
           
        }
        action
        
        {
            theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
            theWalkToTargetSkill(Pose2f(1.f, 1.f, 1.f), Pose2f(theFieldBall.positionRelative.angle()));
        }
    }

    state(moveWalk)
    {
        transition
        {
         
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            if(!theLibCheck.opponentNearTeammate(theRobotPose.translation.x(), theRobotPose.translation.y(), 750)) {
                goto receivePass;
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((theRobotPose.translation-theLibCheck.getJollyPosition()).norm() < 100.f)
                goto turnToBall;
        }
        action
        {
            theLookForwardSkill();
            std::vector<Pose2f> freeAreas;
            for(const auto& cel : theGlobalFieldCoverage.grid) {    
                if(!theLibCheck.opponentOnArea(cel.positionOnField.x(), cel.positionOnField.y())){
                    if(theFieldBall.positionOnField.y() < 0) {
                        if(cel.positionOnField.x() < theRobotPose.translation.x()-500 && cel.positionOnField.x() > theRobotPose.translation.x()-1000 && cel.positionOnField.y() < 0 && cel.positionOnField.y() > theRobotPose.translation.y()-500 && cel.positionOnField.y() < theRobotPose.translation.y()+500){
                            freeAreas.push_back(Pose2f(cel.positionOnField.x(), cel.positionOnField.y()));
                        } else {
                            if(cel.positionOnField.x() < theRobotPose.translation.x()-500 && cel.positionOnField.x() > theRobotPose.translation.x()-1000 && cel.positionOnField.y() > 0 && cel.positionOnField.y() > theRobotPose.translation.y()-500 && cel.positionOnField.y() < theRobotPose.translation.y()+500) {
                            freeAreas.push_back(Pose2f(cel.positionOnField.x(), cel.positionOnField.y()));
                        }
                        }
                    }
                }
                if(freeAreas.size() > 0) {
                    break;
                }
                
            }
            
            if(freeAreas.size() > 0){
                float x,y;
            x=freeAreas.at(0).translation.x();
            y=freeAreas.at(0).translation.y();
                theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),Pose2f(x, y));
            } else {
                theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f),theLibCheck.getJollyPosition());
            }
           }
    }


   
state(turnToBall)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCheck.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((std::abs(theLibCheck.radiansToDegree(theLibCheck.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 10.f))
                goto stand;
        }
        action
        {
            
            theLookForwardSkill();
            theWalkAtRelativeSpeedSkill(Pose2f(20.f, 0.0001f,0.0001f));

        }
    }

    
  }
};

MAKE_CARD(jollyPositionAndUnmarkedCard);
