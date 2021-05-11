(in-package :pp-tut)

(defparameter *final-object-destination*
 (cl-transforms-stamped:make-pose-stamped
  "map" 0.0
  (cl-transforms:make-3d-vector -0.8 2 0.9)
  (cl-transforms:make-identity-rotation)))

(defparameter *base-pose-near-table*
 (cl-transforms-stamped:make-pose-stamped
  "map" 0.0
  (cl-transforms:make-3d-vector -1.447d0 -0.150d0 0.0d0)
  (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))

(defparameter *downward-look-coordinate*
 (cl-transforms-stamped:make-pose-stamped
  "base_footprint" 0.0
  (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
  (cl-transforms:make-identity-rotation)))

(defparameter *base-pose-near-counter*
 (cl-transforms-stamped:make-pose-stamped
  "base_footprint" 0.0
  (cl-transforms:make-3d-vector -0.150d0 2.0d0 0.0d0)
  (cl-transforms:make-quaternion 0.0d0 0.0d0 -1.0d0 0.0d0)))

(defparameter *left-downward-look-coordinate*
 (cl-transforms-stamped:make-pose-stamped
  "base_footprint" 0.0
  (cl-transforms:make-3d-vector 0.65335d0 0.76d0 0.758d0)
  (cl-transforms:make-identity-rotation)))

(defparameter *right-downward-look-coordinate*
 (cl-transforms-stamped:make-pose-stamped
  "base_footprint" 0.0
  (cl-transforms:make-3d-vector 0.65335d0 -0.76d0 0.758d0)
  (cl-transforms:make-identity-rotation)))


;;spawn-bottle to be used in the first exercise, i.e. successfully picking up the bottle
;;--------------------------------------------------------------------------------------

(defun spawn-bottle ()
 (unless (assoc :bottle btr::*mesh-files*)
   (add-objects-to-mesh-list))
 (btr-utils:spawn-object 'bottle-1 :bottle :color '(1 0 0) :pose '((-1.6 -0.9 0.82) (0 0 0 1)))
 (btr:simulate btr:*current-bullet-world* 10))


;;spawn-bottle which can be used in the first exercise on recovering from failures
;;--------------------------------------------------------------------------------

(defun spawn-bottle2 ()
 (unless (assoc :bottle btr::*mesh-files*)
   (add-objects-to-mesh-list))
 (btr-utils:spawn-object 'bottle-1 :bottle :color '(1 0 0) :pose '((-2 -0.9 0.860) (0 0 0 1)))
 (btr:simulate btr:*current-bullet-world* 10))
 
 
;;spawn-bottle which can be used in the second exercise on recovering from failures
;;---------------------------------------------------------------------------------

(defun spawn-bottle3 ()
 (unless (assoc :bottle btr::*mesh-files*)
   (add-objects-to-mesh-list))
 (btr-utils:spawn-object 'bottle-1 :bottle :color '(1 0 0) :pose '((-1.1 -0.75 0.860) (0 0 0 1)))
 (btr:simulate btr:*current-bullet-world* 10))


;;move-bottle which can be used in the first exercise successfully picking up a bottle
;;------------------------------------------------------------------------------------

(defun move-bottle ()
 (spawn-bottle)
 (pr2-proj:with-simulated-robot
   (let ((?navigation-goal *base-pose-near-table*))
     (cpl:par
       (exe:perform (desig:a motion 
                             (type moving-torso)
                             (joint-angle 0.3)))
       (pp-plans::park-arms)
       ;; Moving the robot near the table.
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?navigation-goal)))))))
   ;; Looking towards the bottle before perceiving.
   (let ((?looking-direction *downward-look-coordinate*))
     (exe:perform (desig:a motion 
                           (type looking)
                           (target (desig:a location 
                                            (pose ?looking-direction))))))
   ;; Detect the bottle on the table.
   (let ((?grasping-arm :right)
         (?perceived-bottle (exe:perform (desig:a motion
                                                  (type detecting)
                                                  (object (desig:an object 
                                                                    (type :bottle)))))))
     ;; Pick up the bottle
     (exe:perform (desig:an action
                            (type picking-up)
                            (arm ?grasping-arm)
                            (grasp left-side)
                            (object ?perceived-bottle)))
     (pp-plans::park-arms :arm ?grasping-arm)
     ;; Moving the robot near the counter.
     (let ((?nav-goal *base-pose-near-counter*))
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?nav-goal))))))
     (coe:on-event (make-instance 'cpoe:robot-state-changed))
     ;; Setting the bottle down on the counter
     (let ((?drop-pose *final-object-destination*))
       (exe:perform (desig:an action
                              (type placing)
                              (arm ?grasping-arm)
                              (object ?perceived-bottle)
                              (target (desig:a location 
                                               (pose ?drop-pose))))))
     (pp-plans::park-arms :arm ?grasping-arm))))


;;move-bottle which can be used in the first exercise on  recovering from failures
;;--------------------------------------------------------------------------------

(defun move-bottle2 ()
 (spawn-bottle2) ;NB spawning second bottle
 (pr2-proj:with-simulated-robot
   (let ((?navigation-goal *base-pose-near-table*))
     (cpl:par
       (exe:perform (desig:a motion 
                             (type moving-torso)
                             (joint-angle 0.3)))
       (pp-plans::park-arms)
       ;; Moving the robot near the table.
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?navigation-goal)))))))
   ;; Find and detect the bottle on the table.
   (multiple-value-bind (?perceived-bottle ?grasping-arm) 
       (find-object :bottle)
     (exe:perform (desig:an action
                            (type picking-up)
                            (arm ?grasping-arm)
                            (grasp left-side)
                            (object ?perceived-bottle)))
     (pp-plans::park-arms :arm ?grasping-arm)
     ;; Moving the robot near the counter.
     (let ((?nav-goal *base-pose-near-counter*))
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?nav-goal))))))

     (coe:on-event (make-instance 'cpoe:robot-state-changed))
     ;; Setting the object down on the counter
     (let ((?drop-pose *final-object-destination*))
       (exe:perform (desig:an action
                              (type placing)
                              (arm ?grasping-arm)
                              (object ?perceived-bottle)
                              (target (desig:a location 
                                               (pose ?drop-pose))))))
     (pp-plans::park-arms :arm ?grasping-arm))))


;;move-bottle which can be used in the second exercise on recovering from failures
;;--------------------------------------------------------------------------------

(defun move-bottle3 ()
 (spawn-bottle3) ;NB spawning third bottle
 (pr2-proj:with-simulated-robot
   (let ((?navigation-goal *base-pose-near-table*))
     (cpl:par
       (exe:perform (desig:a motion 
                             (type moving-torso) 
                             (joint-angle 0.3)))
       (pp-plans::park-arms)
       ;; Moving the robot near the table.
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?navigation-goal)))))))

   (multiple-value-bind (?perceived-bottle ?grasping-arm) 
       (find-object :bottle)
     (setf ?grasping-arm (pick-up-object ?perceived-bottle :bottle ?grasping-arm))
     (pp-plans::park-arms :arm ?grasping-arm)
     ;; Moving the robot near the counter.
     (let ((?nav-goal *base-pose-near-counter*))
       (exe:perform (desig:a motion
                             (type going)
                             (target (desig:a location 
                                              (pose ?nav-goal))))))

     (coe:on-event (make-instance 'cpoe:robot-state-changed))
     ;; Setting the object down on the counter
     (let ((?drop-pose *final-object-destination*))
       (exe:perform (desig:an action
                              (type placing)
                              (arm ?grasping-arm)
                              (object ?perceived-bottle)
                              (target (desig:a location 
                                               (pose ?drop-pose))))))
     (pp-plans::park-arms :arm ?grasping-arm))))


(defun get-preferred-arm-for-direction (direction-looked)
 (let ((preferred-arm :RIGHT))
   (when (eq direction-looked *left-downward-look-coordinate*)
     (setf preferred-arm :LEFT))
   preferred-arm))

(defun find-object (?object-type)
 (let* ((possible-look-directions `(,*downward-look-coordinate*
                                    ,*left-downward-look-coordinate*
                                    ,*right-downward-look-coordinate*))
        (?looking-direction (first possible-look-directions)))
   (setf possible-look-directions (cdr possible-look-directions))
   (exe:perform (desig:a motion 
                         (type looking)
                         (target (desig:a location 
                                          (pose ?looking-direction)))))

   (cpl:with-failure-handling
       ((cram-common-failures:perception-object-not-found (e)
          ;; Try different look directions until there is none left.
          (when possible-look-directions
            (roslisp:ros-warn (perception-failure) "~a~%Turning head." e)
            (exe:perform (desig:a motion 
                                  (type looking) 
                                  (direction forward)))
            (setf ?looking-direction (first possible-look-directions))
            (setf possible-look-directions (cdr possible-look-directions))
            (exe:perform (desig:a motion 
                                  (type looking)
                                  (target (desig:a location
                                                   (pose ?looking-direction)))))
            (cpl:retry))
          (cpl:fail 'common-fail:looking-high-level-failure)))

     (let ((?perceived-bottle
             (exe:perform (desig:a motion
                                   (type detecting)
                                   (object (desig:an object 
                                                     (type ?object-type)))))))
       (values ?perceived-bottle (get-preferred-arm-for-direction ?looking-direction))))))


(defun pick-up-object (?perceived-object ?object-type ?grasping-arm)
 (let ((?possible-arms '(:right :left)))
   ;;Retry by changing the arm
   (cpl:with-retry-counters ((arm-change-retry 1))
       (cpl:with-failure-handling
           ((common-fail:object-unreachable (e)
              (roslisp:ros-warn (arm-failure) "Manipulation failed: ~a~%" e)
              (cpl:do-retry arm-change-retry
                (setf ?grasping-arm (car (remove ?grasping-arm ?possible-arms)))
                (cpl:retry))
              (roslisp:ros-warn (arm-failures) "No more retries left")))

         ;; Retry by changing the grasp
         (let* ((?possible-grasp
                  (cram-object-interfaces:get-object-type-grasps ?object-type nil nil nil ?grasping-arm))
                (?grasp (cut:lazy-car ?possible-grasp)))
           (cpl:with-retry-counters ((grasp-retries 3))
             (cpl:with-failure-handling
                 (((or cram-common-failures:manipulation-pose-unreachable
                       cram-common-failures:gripper-closed-completely) (e)
                    (roslisp:ros-warn (grasp-failure)
                                      "~a~%Failed to grasp from ~a using ~a arm "
                                      e ?grasp ?grasping-arm)
                    (cpl:do-retry grasp-retries
                      (when (cut:lazy-car ?possible-grasp)
                        (roslisp:ros-info (trying-new-grasp)
                                          "Trying to grasp from ~a using ~a arm"
                                        ?grasp ?grasping-arm)
                        (setf ?possible-grasp (cut:lazy-cdr ?possible-grasp))
                        (pp-plans::park-arms)
                        (setf ?grasp (cut:lazy-car ?possible-grasp))
                        (cpl:retry)))
                    (roslisp:ros-warn (grasp-failures) "No more retries left")
                    (cpl:fail 'common-fail:object-unreachable)))
               ;; Perform the grasp
               (exe:perform (desig:an action
                                      (type picking-up)
                                      (arm ?grasping-arm)
                                      (grasp ?grasp)
                                      (object ?perceived-object)))))))))
 ?grasping-arm)