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
				printf("hello world");
			}
	};
}
//GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
IGNITION_ADD_PLUGIN(
				chimuelo::IgnActorPosesPlugin,
                ignition::gazebo::System,
                chimuelo::IgnActorPosesPlugin::ISystemConfigure)
IGNITION_ADD_PLUGIN_ALIAS(chimuelo::IgnActorPosesPlugin,"IgnActorPosesPlugin")
