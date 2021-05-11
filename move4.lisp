(defun move-bottle3 (bottle-spawn-pose)
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
      (setf ?grasping-arm (pick-up-object2 ?perceived-bottle ?grasping-arm))
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


(defun pick-up-object2 (?perceived-object ?grasping-arm)
  (let* ((?possible-grasps '(:left-side :right-side :front :back))
         (?grasp (first ?possible-grasps)))
    (setf ?possible-grasps (rest ?possible-grasps))
    (setq a (make-array '(3)))
    (setf (aref a 0) (aref ?perceived-object 0))
    (setf (aref a 1) (aref ?perceived-object 2))
    (setf (aref a 2) (aref ?perceived-object 1))
    (cpl:with-retry-counters ((arm-change-retry 1))
      ;; Outer handle failure handling arm change
      (handle-failure object-unreachable
          ;; Iner handle-failure handling grasp change
          ((handle-failure (or manipulation-pose-unreachable gripper-closed-completely)
               ;; Try to perform the pick up
               ((perform (an action
                             (type picking-up)
                             (arm ?grasping-arm)
                             (grasp ?grasp)
                             (object a))))
             ;; When pick-up fails this block gets executed
             (format t "Grasp failed! Error: ~a~%Grasp: ~a~%Arm: ~a~%"
                     e ?grasp ?grasping-arm)
             ;; Checks if we have any possible grasps left.
             ;; If yes, then the block nested to it gets executed, which will
             ;; set the grasp that is used to the new value and trigger retry
             (when (first ?possible-grasps)
               (format t "Retyring! Trying to grasp from ~a using ~a arm"
                       ?grasp ?grasping-arm)
               (setf ?grasp (first ?possible-grasps))
               (setf ?possible-grasps (rest ?possible-grasps))
               (park-arms)
               (cpl:retry))
             ;; This will get executed when there are no more elements in the 
             ;; ?possible-grasps list. We print the error message and throw a new error
             ;; which will be caught by the outer handle-failure
             (print  "No more grasp retries left :(")
             (cpl:fail 'object-unreachable)))
 
        ;; This is the failure management of the outer handle-failure call
        ;; It changes the arm that is used to grasp
        (print "Manipulation failed!!")
        (print e)
 
        ;; Here we use the retry counter we defined. The value is decremented automatically
        (cpl:do-retry arm-change-retry
          ;; if the current grasping arm is right set left, else set right
          (setf ?grasping-arm (if (eq ?grasping-arm :right) 
                                  :left
                                  :right))
          (cpl:retry))
        ;; When all retries are exhausted print the error message.
        (print "No more arm change retries left :("))))
  ?grasping-arm)