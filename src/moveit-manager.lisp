;;; Copyright (c) 2016, Mihai Pomarlan <blandc@informatik.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-moveit-manager)

(defclass moveit-goal-specification (manipulation-goal-specification)
  ())

(defmethod mot-man:make-goal-specification ((type (eql :moveit-goal-specification)) &rest args)
  (apply #'make-instance (cons 'moveit-goal-specification args)))

(defmethod mot-man:make-goal-specification ((type (eql 'moveit-goal-specification)) &rest args)
  (apply #'make-instance (cons 'moveit-goal-specification args)))

(defun sanity-check (goal-spec)
  ;; Insert conditions you'd want the goal-spec to satisfy here, so that it is doable by
  ;; MoveIt!. Currently, testing that arm-pose-goals inside goal-spec refer to different
  ;; arms (for now, we disallow goal-specs that give goals for different links on the same
  ;; arm).
  (let* ((arms (mapcar #'side
                       (arm-pose-goals goal-spec)))
         (unique-arms (remove-duplicates arms)))
    (equal (length unique-arms) (length arms))))

(defun plan-trajectory (side link-name pose-stamped ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow start-state touch-links object-names-in-hand hand-link-names)
  (let* ((planning-group (planning-group-name side))
         (log-id (first (cram-language::on-prepare-move-arm
                          link-name pose-stamped
                          planning-group ignore-collisions)))
         (result nil))
    (cpl-impl:with-failure-handling
      ((moveit:no-ik-solution (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "No IK solution found.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:planning-failed (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "Planning failed.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:goal-violates-path-constraints (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "Goal violates path constraints.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:invalid-goal-constraints (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "Invalid goal constraints.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:timed-out (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "Timeout.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:goal-in-collision (f)
         (declare (ignore f))
         (roslisp:ros-error (move arm) "Goal in collision.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'cram-plan-failures:manipulation-pose-unreachable
                :result (list side pose-stamped))))
      (setf result
            (multiple-value-bind (start trajectory)
                                 (moveit:move-link-pose
                                   link-name
                                   planning-group
                                   pose-stamped
                                   :ignore-collisions ignore-collisions
                                   :allowed-collision-objects allowed-collision-objects
                                   :collidable-objects collidable-objects
                                   :max-tilt max-tilt
                                   :raise-elbow raise-elbow
                                   :start-state start-state
                                   :touch-links touch-links
                                   :additional-touch-link-groups
                                   (when object-names-in-hand `(,object-names-in-hand))
                                   :additional-collision-objects-groups
                                   (when hand-link-names `(,hand-link-names))
                                   :plan-only t
                                   :additional-values
                                   (when object-names-in-hand `(t)))
              (declare (ignorable start))
              (cram-language::on-finish-move-arm log-id t)
              trajectory)))))

(defun plan-trajectory-sequence (side link-name poses ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow)
  (let* ((touch-links (arm-link-names side))
         (object-names-in-hand (object-names-in-hand side))
         (hand-link-names (hand-link-names side))
         (start-state (second (first (moveit:get-planning-scene-info :robot-state T))))
         (trajectories (mapcar (lambda (pose)
                                 (let* ((trajectory (plan-trajectory side link-name pose ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow start-state
                                                    touch-links object-names-in-hand hand-link-names)))
                                   (setf start-state (moveit:get-end-robot-state start-state trajectory))
                                   trajectory))
                               poses)))
    (moveit:concatenate-trajectories trajectories :ignore-va T)))

(defun plan-trajectories (goal-spec)
  (let* ((keys (keys goal-spec))
         (ignore-collisions (cadr (assoc :ignore-collisions keys)))
         (allowed-collision-objects (cadr (assoc :allowed-collision-objects keys)))
         (collidable-objects (cadr (assoc :collidable-objects keys)))
         (max-tilt (cadr (assoc :max-tilt keys)))
         (raised-elbows (cadr (assoc :raise-elbow keys))))
    (mapcar (lambda (arm-pose-goal)
              (plan-trajectory-sequence (side arm-pose-goal)
                                        (arm-link arm-pose-goal)
                                        (poses arm-pose-goal)
                                        ignore-collisions
                                        allowed-collision-objects
                                        collidable-objects
                                        max-tilt
                                        (if (find (side arm-pose-goal) raised-elbows) (side arm-pose-goal) nil)))
            (arm-pose-goals goal-spec))))

(defmethod mot-man:execute-arm-action ((goal-specification moveit-goal-specification))
  (unless (cadr (assoc :quiet (keys goal-specification)))
    (roslisp:ros-info (moveit motion manager) "Executing arm movement"))
  (if (sanity-check goal-specification)
    ;; TODO: something is weird when concatenating trajectories in cram-moveit, and results
    ;; in occasional controller aborts. Fix that in the future, meanwhile work-around
    (let* (;;(trajectories (plan-trajectories goal-specification))
           (trajectories nil)
           (start-state (second (first (moveit:get-planning-scene-info :robot-state T))))
           (arm-pose-goals (arm-pose-goals goal-specification))
           (plan-only (plan-only goal-specification))
           (keys (keys goal-specification))
           (ignore-collisions (cadr (assoc :ignore-collisions keys)))
           (allowed-collision-objects (cadr (assoc :allowed-collision-objects keys)))
           (collidable-objects (cadr (assoc :collidable-objects keys)))
           (max-tilt (cadr (assoc :max-tilt keys)))
           (raised-elbows (cadr (assoc :raise-elbow keys)))
  ;; TODO: add log call here, around the block with trajectory execution.
           (result 
             (block execute-trajectories
               (loop for arm-goals in arm-pose-goals do
                 (let* ((arm (side arm-goals))
                        (link-name (arm-link arm-goals))
                        (poses (poses arm-goals))
                        (raise-elbow (if (find arm raised-elbows) T nil))
                        (touch-links (arm-link-names arm))
                        (object-names-in-hand (object-names-in-hand arm))
                        (hand-link-names (hand-link-names arm)))
                   (loop for pose in poses do
                     (cpl-impl:with-failure-handling
                       ((moveit:control-failed (f)
                          ;; TODO: add more logging here.
                          (declare (ignore f))
                          (cpl-impl:retry)))
                       (setf start-state (second (first (moveit:get-planning-scene-info :robot-state T))))
                       (let* ((trajectory (plan-trajectory arm link-name pose ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow start-state
                                                           touch-links object-names-in-hand hand-link-names)))
                         (setf trajectories (cons trajectory trajectories))
                         (setf start-state (moveit:get-end-robot-state start-state trajectory))
                         (unless plan-only
                           (cpl-impl:with-failure-handling
                             ((moveit:timed-out (f)
                                (declare (ignore f))
                                ;; A bit of a dirty hack to catch a "TIME-OUT" during execution (as opposed to planning) from Moveit.
                                ;; Turns out such an error may not indicate the robot can't reach, so we will retry the plan/execute
                                ;; steps. To do so, change the signal to something the next failure-handling can catch.
                                (error 'moveit:control-failed)))
                             (moveit:execute-trajectory trajectory))
                           (roslisp:wait-duration 0.2)
                           (setf start-state (second (first (moveit:get-planning-scene-info :robot-state T))))))))))                
              (make-instance 'manipulation-result
                             :all-ok T
                             :trajectories (reverse trajectories)))))
      result)
    (call-fallback goal-specification)))

(defun fallback-to-moveit (goal-spec)
  (copy-goal-specification goal-spec 'moveit-goal-specification))

(defmethod make-fallback-converter ((type (eql :moveit-goal-specification)))
  #'fallback-to-moveit)

(defmethod make-fallback-converter ((type (eql 'moveit-goal-specification)))
  #'fallback-to-moveit)


