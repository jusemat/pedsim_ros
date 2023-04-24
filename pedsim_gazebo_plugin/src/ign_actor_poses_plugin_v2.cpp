/*
Created on Mon Aug  9

@author: sebmat
*/

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include<pedsim_msgs/TrackedPersons.h>
#include<pedsim_msgs/AgentStates.h>

/*
//HelloWorld example
#include <ignition/gazebo/System.hh>
#include <string>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

namespace chimuelo
{
class IgnActorPosesPlugin:
    public ignition::gazebo::System,
	public ignition::gazebo::ISystemConfigure
	//public ignition::gazebo::ISystemPostUpdate
	{
	void Configure(const ignition::gazebo::Entity &_entity,
			const std::shared_ptr<const sdf::Element> &_sdf,
			ignition::gazebo::EntityComponentManager &_ecm,
			ignition::gazebo::EventManager &_eventMgr)
			{
				printf("hello world");
			}
	/*public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
            const ignition::gazebo::EntityComponentManager &_ecm)// override;
            {
			  // This is a simple example of how to get information from UpdateInfo.
			  std::string msg = "Hello, world! Simulation is ";
			  if (!_info.paused)
				msg += "not ";
			  msg += "paused.";

			  // Messages printed with ignmsg only show when running with verbosity 3 or
			  // higher (i.e. ign gazebo -v 3)
			  ignmsg << msg << std::endl;

			}
	};
}

IGNITION_ADD_PLUGIN(
    chimuelo::IgnActorPosesPlugin,
    ignition::gazebo::System,
    chimuelo::IgnActorPosesPlugin::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(chimuelo::IgnActorPosesPlugin,"IgnActorPosesPlugin")*/
//using namespace hello_world;
//void IgnActorPosesPlugin::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
//    const ignition::gazebo::EntityComponentManager &/*_ecm*/)
/*{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  // Messages printed with ignmsg only show when running with verbosity 3 or
  // higher (i.e. ign gazebo -v 3)
  ignmsg << msg << std::endl;

}*/

namespace chimuelo
{
    class IgnActorPosesPlugin :
    	public ignition::gazebo::System,
		public ignition::gazebo::ISystemConfigure{
        //public: ActorPosesPlugin() : WorldPlugin(){}
		
        void Configure(//ignition::gazebo::World &_world,//ignition::gazebo::worldEntity &_world
        	const ignition::gazebo::Entity &_entity,
			const std::shared_ptr<const sdf::Element> &_sdf,
			ignition::gazebo::EntityComponentManager &_ecm,
			ignition::gazebo::EventManager &_eventMgr)
			{
            //this->world_ = ignition::gazebo::World(_entity);
            if (!ros::isInitialized()){
                ROS_ERROR("ROS not initialized");
                return;
            }
            rosNode.reset(new ros::NodeHandle("gazebo_client"));
            ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1,boost::bind(&IgnActorPosesPlugin::OnRosMsg, this, _1), ros::VoidPtr(),&rosQueue);
            rosSub = rosNode->subscribe(so);
            rosQueueThread =std::thread(std::bind(&IgnActorPosesPlugin::QueueThread, this));
            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate, this));
        }


        public:
            // call back function when receive rosmsg
            void OnRosMsg( const pedsim_msgs::AgentStatesConstPtr msg) {
//              ROS_INFO ("OnRosMsg ... ");
//                std::string model_name;
//#if GAZEBO_MAJOR_VERSION < 9
//                for(unsigned int mdl = 0; mdl < world_.GetModelCount(); mdl++) {
//#else
                for(unsigned int mdl = 0; mdl < world_.ModelCount(_ecm); mdl++) {
//#endif
					//std::vector<gazebo::Entity> tmp_models;
                    ignition::gazebo::Model  tmp_model;
                    
                    //tmp_models = world_.Models(_ecm);
                    
//#if GAZEBO_MAJOR_VERSION < 9
//                    tmp_model = world_->GetModel(mdl);
//#else
                    //tmp_model = ignition::gazebo::Model(tmp_models.at(mdl));
                    tmp_model = ignition::gazebo::Model(world_.Models(_ecm).at(mdl));
//#endif
                    std::string frame_id;
                    frame_id = tmp_model.Name(_ecm);

                    for (uint actor =0; actor< msg->agent_states.size() ; actor++) {
                        if(frame_id == std::to_string( msg->agent_states[actor].id)  ){
//                            ROS_INFO_STREAM("actor_id: "<< std::to_string( msg->tracks[actor].track_id) );
                            ignition::math::Pose3d gzb_pose;
                            gzb_pose.Pos().Set( msg->agent_states[actor].pose.position.x,
                                                msg->agent_states[actor].pose.position.y,
                                                msg->agent_states[actor].pose.position.z + MODEL_OFFSET);
                            gzb_pose.Rot().Set(msg->agent_states[actor].pose.orientation.w,
                                               msg->agent_states[actor].pose.orientation.x,
                                               msg->agent_states[actor].pose.orientation.y,
                                               msg->agent_states[actor].pose.orientation.z);

                            try{
                                tmp_model.SetWorldPoseCmd(_ecm,gzb_pose);
                            }
                            catch(ignition::common::exception gz_ex){
                                ROS_ERROR("Error setting pose %s - %s", frame_id.c_str(), gz_ex.what());
                            }

                        }
                    }
				}
				//}
          }


        // ROS helper function that processes messages
        private: void QueueThread() {
            static const double timeout = 0.1;
            while (rosNode->ok()) {
                rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

    private:
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Subscriber rosSub;
        ros::CallbackQueue rosQueue;
        std::thread rosQueueThread;
        ignition::gazebo::World world_;
        ignition::gazebo::EntityComponentManager _ecm;
        //event::ConnectionPtr updateConnection_;
        const float MODEL_OFFSET = 0.75;

    };
}
//GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
IGNITION_ADD_PLUGIN(
				chimuelo::IgnActorPosesPlugin,
                ignition::gazebo::System,
                chimuelo::IgnActorPosesPlugin::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(chimuelo::IgnActorPosesPlugin,"IgnActorPosesPlugin")
