import { Injectable } from '@angular/core';
import { Subject, Observable, BehaviorSubject } from 'rxjs';


declare var ROSLIB: any;

@Injectable({
  providedIn: 'root'
})
export class RosService {

  ros: any;

  private _sim_status: Subject<string> = new BehaviorSubject<string>("");

  public readonly sim_status: Observable<string> = this._sim_status.asObservable();

  constructor() {
    // ROS PLANNER
    this.ros = new ROSLIB.Ros();
    // If there is an error on the backend, an 'error' emit will be emitted.
    this.ros.on('error', function(error) {
      console.log(error);
    });
    // Find out exactly when we made a connection.
    this.ros.on('connection', function() {
      console.log('Connection made!');
    });
    this.ros.on('close', function() {
      console.log('Connection closed.');
    });
    // Create a connection to the rosbridge WebSocket server.
    this.ros.connect("ws://localhost:9090");
  }

  unpausePhysics(callback) {
    console.log(this.ros);
    var UnPausePhysics = new ROSLIB.Service({
      ros : this.ros,
      name : 'gazebo/unpause_physics',
      serviceType : 'std_srvs/Empty'
    });
    var request = new ROSLIB.ServiceRequest({});
    UnPausePhysics.callService(request, function(result) {
      console.log('unpaused physics');
      callback();
    });
  }

  movePart(part: string, object_pose: any, callback=()=>{}) {
    var MovePart = new ROSLIB.Service({
      ros : this.ros,
      name : 'gazebo/set_model_state',
      messageType : '/gazebo_msgs/SetModelState'
    });
    //first element of position array is the timestamp of the movement
    var request = new ROSLIB.ServiceRequest({
      model_state: {
        model_name: part,
        pose: { position: { x: object_pose.position.x, y: object_pose.position.y, z: object_pose.position.z}, orientation: {x: object_pose.orientation.x, y: object_pose.orientation.y, z: object_pose.orientation.z, w: object_pose.orientation.w} },
        twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } },
        reference_frame: 'world'
      }
    });

    MovePart.callService(request, (result)=> {
      // console.log(request);
      setTimeout(()=>{ callback(); }, 500);
    });
    //callback();
  }

  getPart(part: string, callback=(arg)=>{}) {
    var GetPart = new ROSLIB.Service({
      ros : this.ros,
      name : 'gazebo/get_model_state',
      serviceType : '/gazebo_msgs/GetModelState'
    });

    var request = new ROSLIB.ServiceRequest({
      model_name: part,
      relative_entity_name: 'world'
    });
    GetPart.callService(request, (result) => {
      callback(result);
    });
  }

  deletePart(part: string, callback) {
    var DeletePart = new ROSLIB.Service({
      ros: this.ros,
      name : 'gazebo/delete_model',
      serviceType : '/gazebo_msgs/DeleteModel'
    })

    var request = new ROSLIB.ServiceRequest({
      model_name: part
    });
    console.log("Have to delete part");

    DeletePart.callService(request, (result) => {
      console.log(result);
      callback();
    });
  }

  spawnPart(part: string, part_sdf: string, position: string[]) {
    var SpawnPart = new ROSLIB.Service({
      ros: this.ros,
      name : 'gazebo/spawn_sdf_model',
      serviceType : '/gazebo_msgs/SpawnModel'
    });

    console.log("Have to spawn part");
    var request = new ROSLIB.ServiceRequest({
      model_name: part,
      model_xml: part_sdf,
      initial_pose: { position: { x: +position[1], y: +position[2], z: +position[3]}, orientation: {x: +position[4], y: +position[5], z: +position[6], w: +position[7]} },
    });

    SpawnPart.callService(request, (result) => {
      console.log(result);
    });
  }

  pausePhysics() {
    console.log(this.ros);
    var PausePhysics = new ROSLIB.Service({
      ros : this.ros,
      name : 'gazebo/pause_physics',
      serviceType : 'std_srvs/Empty'
    });
    var request = new ROSLIB.ServiceRequest({});
    PausePhysics.callService(request, function(result) {
      console.log('paused physics');
    });
  }

  playSimulation() {
    var playSim = new ROSLIB.Topic({
      ros: this.ros,
      name: '/play_simulation',
      messageType : 'std_msgs/String'
    });

    playSim.publish({data: 'begin'});
  }

  simStatus() {
    var sim_status_listener = new ROSLIB.Topic({
      ros : this.ros,
      name : '/simulation_status',
      messageType : 'std_msgs/String'
    });

    return sim_status_listener; /*.subscribe((data)=> {
      this._sim_status.next(data.data);
    });*/
  }

  goHome() {
    var goHome = new ROSLIB.Topic({
      ros: this.ros,
      name: '/go_home',
      messageType : 'std_msgs/String'
    });

    goHome.publish({data: 'begin'});
  }
}
