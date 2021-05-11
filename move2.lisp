;; Solution for choosing the arm based on the position of the robot
(defun get-preferred-arm (?perceived-object)
  (let* ((obj-name (get-obj-name ?perceived-object))
         (obj-pose (get-current-pose-of-object obj-name))
         (robot-transformation (get-robot-transformation))
         (inverse-robot-transform (inverse-transformation robot-transformation))
         ;; Position of the object relative to the robot
         (obj-in-robot-frame (apply-transformation inverse-robot-transform obj-pose))
         (x (get-x-of-pose obj-in-robot-frame))
         (y (get-y-of-pose obj-in-robot-frame)))
         ;; If the object is in front of the robot
    (if (> x 0)
        ;; if the object is on positive y-axis in front of the robot
        ;; then left, else right
        (if (> y 0)
            :left
            :right)
        ;; If the object is on positive y-axis while behind the robot
        ;; then right, else left
        (if (> y 0)
            :right
            :left))))

(defun move-bottle2 (bottle-spawn-pose)
  (spawn-object bottle-spawn-pose)
  (with-simulated-robot
      (cpl:par
        ;; Moving the robot near the table.
        
        (perform (a motion
                    (type moving-torso) 
                    (joint-angle 0.3)))
        (park-arms))
 
    (let ((?perceived-bottle (perceive-bottle))
 
          (?grasping-arm (get-preferred-arm bottle-spawn-pose)))
      ;; We update the value of ?grasping-arm according to what the method used
      (setf ?grasping-arm (pick-up-object ?perceived-bottle ?grasping-arm))
      (park-arm ?grasping-arm)
      ;; Moving the robot near the counter.
      (let ((?nav-goal *base-pose-near-counter*))
        (perform (an action
                     (type going)
                     (target (a location 
                                (pose ?nav-goal))))))
       ;; Setting the object down on the counter
      (let ((?drop-pose *final-object-destination*))
        (perform (an action
                     (type placing)
                     (arm ?grasping-arm)
                     (object ?perceived-bottle)
                     (target (a location 
                                (pose ?drop-pose))))))
      (park-arm ?grasping-arm))))