rostopic pub /planner/input_waypoints carplanner_msgs/OdometryArray "odoms:
- header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  child_frame_id: ''
  pose:
    pose:
      position: {x: ${1-0.4}, y: ${2-0.1}, z: ${3-0.0}}
      orientation: {x: ${4-0.0}, y: ${5-0.0}, z: ${6-0.9}, w: ${7--0.1}}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  twist:
    twist:
      linear: {x: 1.0, y: 0.0, z: 0.0}
      angular: {x: 0.0, y: 0.0, z: 0.0}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
- header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  child_frame_id: ''
  pose:
    pose:
      position: {x: ${8--2.5}, y: ${9--0.6}, z: ${10-0.0}}
      orientation: {x: ${11-0.0}, y: ${12-0.0}, z: ${13-0.9}, w: ${14--0.1}}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  twist:
    twist:
      linear: {x: 1.0, y: 0.0, z: 0.0}
      angular: {x: 0.0, y: 0.0, z: 0.0}
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"