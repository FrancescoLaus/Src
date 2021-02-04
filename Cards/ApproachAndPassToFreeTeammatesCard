#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Tools/Math/BHMath.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"



#include "Platform/SystemCall.h"
#include <string>

CARD(ApproachAndPassToFreeTeammatesCard,
{,
  CALLS(LookForward),
  CALLS(LookAtPoint),
  CALLS(WalkToTarget),
  CALLS(Kick),
  CALLS(GoalTarget),
  CALLS(WalkToTargetPathPlanner),
  CALLS(WalkToTargetPathPlannerStraight),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToApproach),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(Activity),

  REQUIRES(GlobalFieldCoverage),
  REQUIRES(MotionInfo),
  REQUIRES(FieldBall),
  REQUIRES(LibCheck),
  REQUIRES(TeamPlayersModel),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  REQUIRES(BallModel),
  
  USES(BehaviorStatus),
  USES(TeamData),

  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(50) initialWaitTime,
    (int)(3500) ballNotSeenTimeout,
    (Angle)(5_deg) alignThreshold,
    (float)(100.f) ballYThreshold,
    (float)(180.f) ballOffsetX,
    (float)(450.f) ballXnearTh,
    (Rangef)({170.f, 190.f}) ballOffsetXRange,
    (Rangef)({-350.f, 350.f}) approachYRange,
    (Rangef)({170.f, 380.f}) ballOffsetXRange2,
    (Rangef)({-130.f, -180.f}) ballOffsetYRange2,
    (Rangef)({-150.f, 150.f}) smallApproachYRange,
    (Rangef)({150.f, 300.f}) smallApproachXRange,
    (float)(-75.f) ballOffsetY,
    (Rangef)({-85.f, -65.f}) ballOffsetYRange,
    (bool)(true) debugText,
    (Angle)(20_deg) angle_target_treshold,
  }),
});


class ApproachAndPassToFreeTeammatesCard : public ApproachAndPassToFreeTeammatesCardBase
{
  double distanceConfirmed = 0.0;
  bool alreadyEnqueued = false;
  
   bool preconditions() const override
  {
        float x, y;
        x = freeTeammate().translation.x();
        y = freeTeammate().translation.y();
        return (theLibCheck.passLineFree(x,y,theFieldBall.positionOnField.x(), theFieldBall.positionOnField.x()) && !theLibCheck.opponentNearTeammate(x,y,400) && theRobotPose.translation.x() < x+500  && theFieldBall.positionOnField.x() < 3000) || (theFieldBall.positionOnField.x() > 3500 && (theFieldBall.positionOnField.y() < -1500 || theFieldBall.positionOnField.y() > 1500));
        }

  bool postconditions() const override
  {
        float x, y;
        x = freeTeammate().translation.x();
        y = freeTeammate().translation.y();  
      
       return (!theLibCheck.passLineFree(x,y,theFieldBall.positionOnField.x(), theFieldBall.positionOnField.x()) || theLibCheck.opponentNearTeammate(x,y,400) || theRobotPose.translation.x() > x+500) || (theFieldBall.positionOnField.x() > 3000 
               && (theFieldBall.positionOnField.y() > -1500 && theFieldBall.positionOnField.y() < 1500));
 }

  option
  {
    initial_state(start)
    {
      transition
      {

        if(state_time > initialWaitTime)
        {
          goto turnToBall;
        }
      }

      action
      {        
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
        {
          goto turnToBall;
        }
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

   
    state(turnToBall)
    {
      transition
      { 
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        if(std::abs(theFieldBall.positionRelative.angle()) < alignThreshold)
        {
          goto walkToBall;
        }          
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(approachAndCross) 
    {
      transition
      {
       
       if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }
        
        if(theFieldBall.positionRelative.norm() < 0 ){
          goto turnToBall;
        }
          
        if(ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) 
          && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())){
            goto kick;      
          }


      }

      action
      {
        float x, y;
        x = freeTeammate().translation.x();
        y = freeTeammate().translation.y();
        double distanceTarget = theLibCheck.distance(theRobotPose.translation, Pose2f(x,y));
        distanceConfirmed = distanceTarget+500;
        theLookForwardSkill();
        theWalkToApproachSkill(Vector2f(x,y), ballOffsetX, ballOffsetY, true);
       }
        
    
    }

    

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
        {
          goto searchForBall;
        }


        if(smallApproachXRange.isInside(theFieldBall.positionRelative.x()) 
            && smallApproachYRange.isInside(theFieldBall.positionRelative.y()))
            goto approachAndCross;
      
    }

      action
      {
       theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
      
        theWalkToTargetPathPlannerSkill(Pose2f(1.f,1.f,1.f), Pose2f(theFieldBall.positionOnField) - Pose2f(180.f, 50.f));

      }
    }


    state(kick)
    {
      transition
      {
        if(theMotionInfo.walkRequest.walkKickRequest.kickType != WalkKicks::none || state_time > 2000)
        {
          alreadyEnqueued = false;
          goto start;
        }
           
      }
      action
      {
        theLookAtPointSkill(Vector3f(theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(), 0.f));
        theKickSkill(false, distanceConfirmed, false); 
      }
    }
  }

  Angle calcAngleToTarget(Pose2f target) const
  {
    return (theRobotPose.inversePose * Vector2f(target.translation)).angle();
  }

  Pose2f freeTeammate() const
  { 
    float x, y;
    std::vector<Pose2f> poseTeammates;
    poseTeammates = theLibCheck.findPoseTeammatesOverX(theRobotPose.translation.x());   
    Pose2f nearestTeammatePose = theLibCheck.findNearestFreeTeammate(theRobotPose.translation, poseTeammates);	
    Pose2f poseFreeTeammate = theLibCheck.findPoseFreeTeammates(nearestTeammatePose, poseTeammates, 750);
    x = poseFreeTeammate.translation.x();
    y = poseFreeTeammate.translation.y();
    return Pose2f(x,y);
  }

};

MAKE_CARD(ApproachAndPassToFreeTeammatesCard);
