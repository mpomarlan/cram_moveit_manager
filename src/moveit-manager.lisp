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

(in-package :mot-man)

(defclass moveit-goal-specification (manipulation-goal-specification)
  (()))

(defmethod make-goal-specification ((type (eql :moveit-goal-specification)) &rest args)
  (apply #'make-instance (cons 'moveit-goal-specification args)))

(defmethod make-goal-specification ((type (eql 'moveit-goal-specification)) &rest args)
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
  (let* ((result nil)
         (planning-group (planning-group-name side)))
    (cpl-impl:with-failure-handling
      ((moveit:no-ik-solution (f)
         (declare (ignore f))
         (ros-error (move arm) "No IK solution found.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:planning-failed (f)
         (declare (ignore f))
         (ros-error (move arm) "Planning failed.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:goal-violates-path-constraints (f)
         (declare (ignore f))
         (ros-error (move arm) "Goal violates path constraints.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:invalid-goal-constraints (f)
         (declare (ignore f))
         (ros-error (move arm) "Invalid goal constraints.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:timed-out (f)
         (declare (ignore f))
         (ros-error (move arm) "Timeout.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
                :result (list side pose-stamped)))
       (moveit:goal-in-collision (f)
         (declare (ignore f))
         (ros-error (move arm) "Goal in collision.")
         (cram-language::on-finish-move-arm log-id nil)
         (error 'manipulation-pose-unreachable
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
                                   :additional-touch-link-groups `(,object-names-in-hand)
                                   :additional-collision-objects-groups `(,hand-link-names)
                                   :plan-only T
                                   :additional-values `(t))
              (declare (ignorable start))
              trajectory))))

(defun plan-trajectory-sequence (side link-name poses ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow)
  (let* ((touch-links (arm-link-names side))
         (object-names-in-hand (object-names-in-hand side))
         (hand-link-names (hand-link-names side))
         (start-state nil)
         (trajectories (mapcar (lambda (pose)
                                 (let* ((trajectory (plan-trajectory side link-name pose ignore-collisions allowed-collision-objects collidable-objects max-tilt raise-elbow start-state
                                                    touch-links object-names-in-hand hand-link-names)))
                                   ;; TODO: implement the moveit:get-end-robot-state function!
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
                                        raise-elbow (if (find (side arm-pose-goal) raised-elbows) T nil)))
            (arm-pose-goals goal-spec))))

(defmethod execute-arm-action ((goal-specification moveit-goal-specification))
  (unless (cadr (assoc :quiet (keys goal-specification)))
    (ros-info (moveit motion manager) "Executing arm movement"))
  (if (sanity-check goal-specification)
    (let* ((trajectories (plan-trajectories goal-specification))
           (plan-only (plan-only goal-specification))
  ;; TODO: add log call here, around the block with trajectory execution.
           (result 
             (block execute-trajectories
              (cpl-impl:with-failure-handling
                ((moveit:control-failed (f)
                   ;; TODO: add more logging here.
                   ;; TODO: think about whether this is useful, ie. consider just propagating this signal upwards. The motion managers will likely be using the same
                   ;; low level robot controller, and just switching to another motion manager is unlikely to help.
                   (roslisp:ros-error (moveit motion manager) "Control failure while executing moveit trajectories, attempting to fall-back to another motion manager.")
                   (return-from execute-trajectories (make-instance 'manipulation-result
                                                                    :all-ok nil
                                                                    :trajectories trajectories
                                                                    :error-object f
                                                                    :error-message "Control failure occurred while executing moveit trajectories."))))
                (unless plan-only
                  (moveit:execute-trajectories trajectories :ignore-va T))
                (make-instance 'manipulation-result
                               :all-ok T
                               :trajectories trajectories)))))
      (if (all-ok result)
        result
        (call-fallback goal-specification)))
    (call-fallback goal-specification)))

(defun fallback-to-moveit (goal-spec)
  (copy-goal-specification goal-spec 'moveit-goal-specification))

