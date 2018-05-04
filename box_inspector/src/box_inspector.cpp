//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  	

  	//WRITE ME!!!
  	//ROS_WARN("NEED TO WRITE update_inspection() ");


  	got_new_snapshot_=false;
  	while(!got_new_snapshot_) {
	    ros::spinOnce(); // refresh camera image
	    ros::Duration(0.5).sleep();
	    ROS_INFO("waiting for logical camera image");
  	}
  	ROS_INFO("got new image");


  	int num_box =  box_inspector_image_.models.size();
  	int num_order = desired_models_wrt_world.size();

    ROS_WARN("num_box: %i", num_box);
    ROS_WARN("num_order: %i", num_order);

  	string box_name("shipping_box");
  	ROS_INFO("update_inspection: box camera saw %d objects",num_box);
  	orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  	satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  	misplaced_models_actual_coords_wrt_world.clear();
  	misplaced_models_desired_coords_wrt_world.clear();
  	missing_models_wrt_world.clear();

/*
original plan:
1. for every object on the order, are they in the box? 
    write down all the no
2. for every object in the box, are they on the order? 
    write down all the no
3. for all the yes in 2, are they in the right position? 
    all yes are good, all no are wrong location

fixed code:
1. picked out all the satisfied product 
2. mark true on the ones that are not orphan, misplaced or missing
3. lopping through all in order & box, and pick out all the orphan, misplaced and missing objects without marks

*/

  	vector<osrf_gear::Model> real_models_wrt_world;
  	osrf_gear::Shipment shipment_status;
  	geometry_msgs::PoseStamped box_pose_wrt_world;
  	model_poses_wrt_box(shipment_status);
  	get_box_pose_wrt_world(box_pose_wrt_world);
  	compute_shipment_poses_wrt_world(shipment_status, box_pose_wrt_world, real_models_wrt_world);

	  osrf_gear::Model box;
	  osrf_gear::Model order;

  	// for missing & orphan & misplace
  	bool boxes[num_box]; 
  	bool orders[num_order]; 
    bool boxsatisfied[num_box]; 
    bool ordersatisfied[num_box]; 
  	for (bool i : boxes) i = false;
	  for (bool i : orders) i = false;
    for (bool i : boxsatisfied) i = false;
    for (bool i : ordersatisfied) i = false;

  	for (int i = 0; i < num_box - 1; i++) {
	    box = real_models_wrt_world[i];
	    string box_name(box.type);

	    for (int j = 0; j < num_order; j++){ 
	      	order = desired_models_wrt_world[j];
	      	string order_name(order.type);

	      	if (box_name.compare(order_name) == 0) {

	      		// for 1. missing & 2. orphan, actul push back happen at the end of this method
	      		boxes[i] = true;
	      		orders[j] = true;

            // for 3. misplaced, actul push back happen at the end of this method
		      	float dx = abs(box.pose.position.x - order.pose.position.x);
		      	float dy = abs(box.pose.position.y - order.pose.position.y);
		      	float dz = abs(box.pose.position.z - order.pose.position.z);

		      	float o = abs(xformUtils_.convertPlanarQuat2Phi(box.pose.orientation) - 
		      		xformUtils_.convertPlanarQuat2Phi(order.pose.orientation));
		      	
            // 4. all good (satisfied)
			      if (dx < 0.03 && dy < 0.03 && dz < 0.03 && o < 0.1 ) {
		        
              boxsatisfied[i] = true;
              ordersatisfied[j] = true;
		        	satisfied_models_wrt_world.push_back(box); 

		      	}
		  	  }
	  	}
	  }


  	for (int j = 0; j < num_order; j++){ 
  		order = desired_models_wrt_world[j];

      // 1. missing models (missing); in order & not in box
  		if (orders[j] == false) {
  			missing_models_wrt_world.push_back(order);
  		}

      // 3. models in order with wrong position and orientation (misplaced)
      else if (ordersatisfied[j] == false) {   
        misplaced_models_desired_coords_wrt_world.push_back(order);
      }
  	}


  	for (int i = 0; i < num_box - 1; i++) { // exclude box
  		box = real_models_wrt_world[i];

      // 2. models do not belong (orphan); in box & not in order
  		if (boxes[i] == false) {
  			orphan_models_wrt_world.push_back(box);
  		}

      // 3. models in box with wrong position and orientation (misplaced)
      else if (boxsatisfied[i] == false) {
        misplaced_models_actual_coords_wrt_world.push_back(box);
      }
  	}

    /*
  	ROS_WARN("Orphan!");
  	for (std::vector<osrf_gear::Model>::const_iterator i = orphan_models_wrt_world.begin(); i != orphan_models_wrt_world.end(); ++i)
      	std::cout << *i << ' ';
    ROS_WARN("Missing!");
    for (std::vector<osrf_gear::Model>::const_iterator i = missing_models_wrt_world.begin(); i != missing_models_wrt_world.end(); ++i)
      	std::cout << *i << ' ';
    ROS_WARN("Misplaced!");
    for (std::vector<osrf_gear::Model>::const_iterator i = misplaced_models_actual_coords_wrt_world.begin(); i != misplaced_models_actual_coords_wrt_world.end(); ++i)
      	std::cout << *i << ' ';
    ROS_WARN("Satisfied!!!");
    for (std::vector<osrf_gear::Model>::const_iterator i = satisfied_models_wrt_world.begin(); i != satisfied_models_wrt_world.end(); ++i)
      	std::cout << *i << ' ';
    */
}

/**

  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
  for (int i=0;i<num_parts_seen;i++) {
     orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
  }
  
**/


//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
