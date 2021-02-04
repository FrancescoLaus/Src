  libCheck.passLineFree = [&](float x, float y, float xBallPosition, float yBallPosition) -> bool
  {
      float xCart, yCart, m, q, xCartPos, yCartPos, y2;
      std::vector<Vector2f> vec;
      xCart = yBallPosition;
      yCart = xBallPosition;
      xCartPos = y;
      yCartPos = x;
      if(yCartPos > -4000) {
        if(xCartPos < 0) {
          xCartPos=std::abs(xCartPos);
        } else {
          xCartPos = -xCartPos;
        }
        if(xCart < 0) {
          xCart=std::abs(xCart);
        } else {
          xCart = -xCart;
        }
        m = ((yCartPos - yCart) / (xCartPos-xCart));
        q = yCart - m*xCart;
        if(xCartPos > xCart) {
          for(float pos = xCart ; pos < xCartPos; pos = pos+20) {
              y2 = (pos*m) + q;
              vec.push_back(Vector2f((abs(y2)), -pos));
          } 
        } else {
          for(float pos = xCart; pos > xCartPos; pos = pos-20) {
              y2 = (pos*m) + q;
              vec.push_back(Vector2f((y2), -pos));
            }
        }     
       }
    bool canPass = true;
    for(int i = 0; i< vec.size(); i++) {  
      if(libCheck.opponentNearTeammate(vec.at(i).x(), vec.at(i).y(), 200)) {
        canPass = false;
      break;
      }
    }
  return canPass;
  };

  libCheck.getJollyPosition = [&] () -> Vector2f
  {
    // to return
    Vector2f jollyPosition = Vector2f(0,0);

    // take ball position
    Vector2f ballPosition;
    ballPosition = theTeamBallModel.position;

    if(ballPosition.y() < 1250.f && ballPosition.y() > -1250) {
      if(theRobotPose.translation.y() < 0.f) {
        if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
          jollyPosition.y() = theFieldDimensions.yPosRightSideline + 1500.f;
         } else {
           jollyPosition.y() = theFieldDimensions.yPosRightSideline + 1000.f;
          }
      } else {
        if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
          jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 2500.f;
         } else {
           jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 1000.f;
          }

      }
    } else {
      jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 2750.f;
    }

    // if the ball is not ahead of the previous jolly position by 1m
    // take previous y
    for(const auto& mate : theTeamData.teammates) {
      if(mate.role == Role::RoleType::jolly) {
        // check jolly position w.r.t. ball
        if(ballPosition.x()+500.f < 0.f) {
          jollyPosition.y() = -1500.f;
        } else if(ballPosition.x() > mate.theRobotPose.translation.x()+1000.f) {
          jollyPosition.y() = mate.theRobotPose.translation.y();
        }
      }
    }

    
    //jollyPosition.x() = theFieldDimensions.xPosOpponentGroundline*0.6f;
    Vector2f strikerPosition;
         for(const auto& teammate : theTeamData.teammates){
           if(teammate.role == Role::RoleType::striker) {
              strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
              
          }
          }
          jollyPosition.x() = strikerPosition.x() + 2200.f;
          if(jollyPosition.x() > (theFieldDimensions.xPosOpponentGroundline*0.6f)) {
            return Vector2f(theFieldDimensions.xPosOpponentGroundline*0.6f+300.f, jollyPosition.y());
          } 
          if(strikerPosition.x() < theFieldDimensions.xPosOwnPenaltyArea + 2000.f) {
            return Vector2f(strikerPosition.x() + 1000, jollyPosition.y());
          }

          return jollyPosition;
          
    Vector2f translation = Vector2f(0.f, 0.f);
    for(unsigned i=0; i<4; ++i) {
      jollyPosition = jollyPosition+translation;
      // compute best passing line
      Eigen::ParametrizedLine<float,2> toBall = Eigen::ParametrizedLine<float,2>::Through(jollyPosition, ballPosition);

      float minDist = std::numeric_limits<float>::max();
      for(auto obs : theTeamPlayersModel.obstacles) {
        if(obs.center.x() > ballPosition.x() && obs.center.x() < jollyPosition.x()){
          if(obs.isOpponent()) {
            float thisDist = toBall.distance(obs.center);
            if(thisDist < minDist)
              minDist = thisDist;
          }
        }
      }

      if(minDist > 500.f) {
        return jollyPosition;
      }

       //increment translation
      translation.x() = translation.x()-300.f;
    }


   return jollyPosition;
  };

  libCheck.opponentOnOurField = [&] () -> bool
  {
    for(auto const& obs : theTeamPlayersModel.obstacles){
        if(obs.isOpponent() && obs.center.x() < 0.f){
            return true;
        }
    }
    return false;
  };

   libCheck.OpponentInFrontOfMe = [&] (float x) -> int
  {
    int opponent = 0;
    for(auto const& obs : theTeamPlayersModel.obstacles) {
  		if(obs.isOpponent() && obs.center.x() > x) {
  			opponent += 1;
  		} 	
    }
    return opponent;
  };

  libCheck.opponentNearTeammate = [&] (float x, float y, float d) -> bool
  {
    for(auto const& obs : theTeamPlayersModel.obstacles) {
  		if(obs.isOpponent() && (obs.center.x() < x+d && obs.center.x() > x-d) && (obs.center.y() < y+d && obs.center.y() > y-d)) {
  			return true;
  		} 	
    }
    return false;
  };

  libCheck.opponentOnArea = [&] (float xArea, float yArea) -> bool
  {
  	for(auto const& obs : theTeamPlayersModel.obstacles) {
  		if(obs.isOpponent() && obs.center.x() < xArea  && obs.center.y() < yArea/*+1000*/) {
  			return true;
  		} 	
}

  return false;
};
  
  libCheck.teammateOnArea = [&] (float xArea, float yArea) -> bool
  {
  	for(auto const& obs : theTeamPlayersModel.obstacles) {
  		if(obs.isTeammate() && obs.center.x() < xArea && obs.center.y() < yArea) {
  			return true;
  		} 	
}

  return false;
};

 
  libCheck.findPoseTeammatesOverX = [&] (float x) -> std::vector<Pose2f> {
    std::vector<Pose2f> poseTeammate;
        for(const auto& mate : theTeamData.teammates) {
          if(mate.theRobotPose.translation.x() > x) {
            poseTeammate.push_back(Pose2f(mate.theRobotPose.translation.x(), mate.theRobotPose.translation.y()));
          }
        }
        if(poseTeammate.empty()) {
         for(const auto& mate : theTeamData.teammates) {
          if(mate.theRobotPose.translation.x() < x) {
            poseTeammate.push_back(Pose2f(mate.theRobotPose.translation.x(), mate.theRobotPose.translation.y()));
          }
        } 
        }
        
        return poseTeammate;   
  };

  libCheck.findNearestFreeTeammate = [&] (Pose2f striker, std::vector<Pose2f> poseTeammates) -> Pose2f {
    float x,y;
    float min = libCheck.distance(striker.translation, poseTeammates.at(0).translation);
    x=poseTeammates.at(0).translation.x();
    y=poseTeammates.at(0).translation.y();
    for(int i = 1; i < poseTeammates.size(); i++) {
      if(libCheck.distance(striker.translation, poseTeammates.at(i).translation) < min) {
        min = libCheck.distance(striker.translation, poseTeammates.at(i).translation);
        x=poseTeammates.at(i).translation.x();
        y=poseTeammates.at(i).translation.y();
      }
    }
    return Pose2f(x,y);
  };

  libCheck.findPoseFreeTeammates = [&] (Pose2f myPose, std::vector<Pose2f> poseTeammate, float d) -> Pose2f {
    float x = myPose.translation.x();
    float y = myPose.translation.y();
    if(libCheck.opponentNearTeammate(x, y, d)) {
            for(int i = 0; i < poseTeammate.size(); i++) {
              if(!libCheck.opponentNearTeammate(poseTeammate.at(i).translation.x(), poseTeammate.at(i).translation.y(), 500)) {
                x= poseTeammate.at(i).translation.x();
                y= poseTeammate.at(i).translation.y();
            }
          }
        }
        return Pose2f(x, y);
  };

  
