/*********************************************************************
*  Script en C++ realizado modificado por Enrique Ortega,
*  cuyo autor original es Eitan Marder-Eppstein
*
*
* Software License Agreement (BSD License)
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*
* For a discussion of this tutorial, please see:
* http://pr.willowgarage.com/wiki/navigation/Tutorials/SendingSimpleGoals
*********************************************************************/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  //MoveBaseClient ac("pose_base_controller");
  //Se especifica el nombre del servidor del actionlib del PMB2
  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  move_base_msgs::MoveBaseGoal goal;

  //Mandamos una meta para que avance 0.5m en X y 0.1m en Y
  //Se puede poner cualquier frame_id, como "map" o "base_link"
  //Si ponemos "base_link" en teoría avanza respecto coordenada de la base
  //	en lugar de mandar coordenadas al mapa
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.position.y = 0.1;
  //Mandar cualquier orientación, aunque sea 0
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hurra, la base se movio 0.5 metros");
  else
    ROS_INFO("La base fallo al alcanzar la meta");

  return 0;
}
