;; Modified find-object which changes torso link during perception failure
 
(defun find-object2 (?object-type)
  (let* ((possible-look-directions `(,*downward-look-coordinate*
                                     ,*left-downward-look-coordinate*
                                     ,*right-downward-look-coordinate*))
         (?looking-direction (first possible-look-directions))
         (possible-torso-link-positions '(0.1 0.2))
         (?current-torso-link-position 0.3))
    (setf possible-look-directions (rest possible-look-directions))
    ;; Look towards the first direction
    (perform (an action
                 (type looking)
                 (target (a location 
                            (pose ?looking-direction)))))
    ;; Set the first torso link angle
    (perform (a motion
                (type moving-torso)
                (joint-angle ?current-torso-link-position)))
    ;; perception-object-not-found is the error that we get when the robot cannot find the object.
    ;; Now we're wrapping it in a failure handling clause to handle it
    (handle-failure object-nowhere-to-be-found
        ((handle-failure perception-object-not-found
             ;; Try the action
             ((perform (an action
                           (type detecting)
                           (object (an object 
                                       (type ?object-type))))))
 
           ;; If the action fails, try the following:
           ;; try different look directions until there is none left.
           (when possible-look-directions
             (print "Perception error happened! Turning head.")
             ;; Resetting the head to look forward before turning again
             (perform (an action
                          (type looking) 
                          (direction forward)))
             (setf ?looking-direction (first possible-look-directions))
             (setf possible-look-directions (rest possible-look-directions))
             (perform (a motion 
                         (type looking)
                         (target (a location
                                    (pose ?looking-direction)))))
             ;; This statement retries the action again
             (cpl:retry))
           ;; If everything else fails, error out
           ;; Reset the neck before erroring out
           (perform (an action
                        (type looking)
                        (direction forward)))      
           (cpl:fail 'object-nowhere-to-be-found)))
      (when possible-torso-link-positions
        (print "Perception error happened! Changing torso link position")
        (setf ?current-torso-link-position (first possible-torso-link-positions))
        (setf possible-torso-link-positions (rest possible-torso-link-positions))
        (setf possible-look-directions `(,*downward-look-coordinate*
                                         ,*left-downward-look-coordinate*
                                         ,*right-downward-look-coordinate*))
        (setf ?looking-direction (first possible-look-directions))
        (perform (a motion 
                    (type moving-torso)
                    (joint-angle ?current-torso-link-position)))
        (cpl:retry))
      (print "All Torso link positions exhausted :(")
      (cpl:fail 'object-nowhere-to-be-found))))


(defun move-bottle2 (bottle-spawn-pose)
  (spawn-object bottle-spawn-pose)
  (with-simulated-robot
      (cpl:par
        ;; Moving the robot near the table.
        
        (perform (a motion
                    (type moving-torso) 
                    (joint-angle 0.3)))
        (park-arms))
 
    (let* ((?perceived-bottle (perceive-bottle))
 
          (?grasping-arm (get-preferred-arm ?perceived-bottle)))
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


(defun perceive-bottle2 ()
;; 
(let ((?possible-look-locations `(,*base-pose-near-and-next-counter*))
                  (?looking-location *base-pose-near-table*))
                  (perform (an action
                     (type going)
                     (target (a location 
                                (pose ?looking-location)))))
              (handle-failure (or object-nowhere-to-be-found
                                  object-unreachable)
 
                  ((find-object2 :bottle))
 
                (when (first ?possible-look-locations)
                  (setf ?looking-location (first ?possible-look-locations))
                  (setf ?possible-look-locations (rest ?possible-look-locations))
                  (perform (an action
                               (type going)
                               (target (a location
                                          (pose ?looking-location)))))
                  (cpl:retry))
                (cpl:fail 'object-unreachable))))