//package org.usfirst.frc.team649.robot;
//
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossible2ndIntakeFront;
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossible2ndIntakeRear;
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeFront;
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleIntakeRear;
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleStoreFront;
//import org.usfirst.frc.team649.robot.CommandGroups.DownAndFlipWhenPossibleStoreRear;
//import org.usfirst.frc.team649.robot.commands.ArmMotionProfile;
//import org.usfirst.frc.team649.robot.commands.LiftMotionProfile;
//import org.usfirst.frc.team649.robot.commands.RunIntakeWheels;
//import org.usfirst.frc.team649.robot.commands.SetIntakePistons;
//import org.usfirst.frc.team649.robot.subsystems.ArmSubsystem;
//import org.usfirst.frc.team649.robot.subsystems.LiftSubsystem;
//
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//public class OldTeleopRun {
//	public void teleopRun() {
//		if (oi.operator.newShiftToggle()) {
//			isHigh = !isHigh;
//			drive.shift(isHigh);
//		}
//		double rot = -oi.driver.getRotation();
//		if (rot > 0) {
//			rot = Math.pow(rot, 1);
//		} else {
//			rot = -Math.pow(Math.abs(rot), 1);
//		}
//		drive.driveFwdRotate(oi.driver.getForward(), rot, true);
//
//		if (oi.operator.getIntakeState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
//					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
//				if (armIsFront) {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState, false).start();
//
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState, false).start();
//
//				}
//			}
//		} else if (oi.operator.getStoreState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
//					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.STORE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.STORE_REAR) {
//				if (armIsFront) {
//					timesCalled++;
//					SmartDashboard.putNumber("times Call", timesCalled);
//					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState, false).start();
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState, false).start();
//				}
//			}
//
//		}
//
//		else if (oi.operator.getScaleLowState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE
//					&& liftState != LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_LOW_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE, liftState, 0).start();
//				SmartDashboard.putBoolean("got in", true);
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
//				if (armIsFront) {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState, false).start();
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState, false).start();
//				}
//			}
//		} else if (oi.operator.getScaleMidState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE
//					&& liftState != LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_MID_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE, liftState, 0).start();
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.MID_DROP_REAR) {
//				if (armIsFront) {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState, false).start();
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState, false).start();
//				}
//			}
//		} else if (oi.operator.getScaleHighState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE
//					&& liftState != LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_HIGH_SCALE_STATE;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE, liftState, 0).start();
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
//				if (armIsFront) {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState, false).start();
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState, false).start();
//				}
//			}
//
//		} else if (oi.operator.getArmUpSmall()) {
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
//					&& arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ < 0) {
//				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
//				if (arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
//					armIsFront = true;
//				} else {
//					armIsFront = false;
//				}
//				customArmPos = arm.getArmRaw() + ArmSubsystem.ArmEncoderConstants.ADJ;
//				new ArmMotionProfile(customArmPos, armState, false).start();
//			}
//		} else if (oi.operator.getArmDownSmall()) {
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN && arm.getArmRaw()
//					- ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) {
//				armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
//				if (arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ > ArmSubsystem.ArmEncoderConstants.MID) {
//					armIsFront = true;
//				} else {
//					armIsFront = false;
//				}
//				customArmPos = arm.getArmRaw() - ArmSubsystem.ArmEncoderConstants.ADJ;
//				new ArmMotionProfile(customArmPos, armState, false).start();
//			}
//		} else if (oi.operator.getLiftUpSmall()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP && lift.getRawLift()
//					+ LiftSubsystem.LiftEncoderConstants.ADJ_DIST < LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_UP;
//				customLiftPos = (int) lift.getRawLift() + LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
//				new LiftMotionProfile(customLiftPos, liftState, 0).start();
//			}
//
//		} else if (oi.operator.getLiftDownSmall()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN
//					&& lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST > 0) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_CUSTOM_STATE_DOWN;
//				customLiftPos = (int) lift.getRawLift() - LiftSubsystem.LiftEncoderConstants.ADJ_DIST;
//				new LiftMotionProfile(customLiftPos, liftState, 0).start();
//			}
//		} else if (oi.operator.getExchangeState()) {
//			if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
//					&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
//				liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
//				new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, liftState, 0).start();
//			}
//			if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR
//					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
//					&& armState != ArmSubsystem.ArmStateConstants.EXCHANGE_REAR) {
//				if (armIsFront) {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState, false).start();
//				} else {
//					armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
//					new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
//				}
//			}
//
//		} else if (oi.operator.flipArm()) {
//			// if(lift.isCarriageAtBottom()){ //temp
//			if (liftState % 2 == 1) {
//				canFlipArm = false;
//			} else {
//				canFlipArm = lift.canFlip();
//			}
//			canFlipArm = true;
//			if (!isOpen && canFlipArm) {
//				if (!prevStateFlipArm && lift.getRawLift() < LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE) {
//					// if (armIsFront) {
//					if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP
//							|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState,
//								false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_REAR, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_REAR, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.STORE_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
//
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_REAR, armState, false).start();
//
//					}
//					// } else {
//					if (armState == ArmSubsystem.ArmStateConstants.EXCHANGE_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_FRONT, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_DOWN
//							|| armState == ArmSubsystem.ArmStateConstants.CUSTOM) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_CUSTOM_UP;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR - customArmPos, armState,
//								false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HIGH_DROP_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_HIGH_DROP_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.HIGH_DROP_FRONT, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.MID_DROP_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_MID_DROP_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.MID_DROP_FRONT, armState, false).start();
//
//					} else if (armState == ArmSubsystem.ArmStateConstants.STORE_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState, false).start();
//
//					} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
//
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState, false).start();
//					} else if (armState == ArmSubsystem.ArmStateConstants.SWITCH_REAR
//							|| armState == ArmSubsystem.ArmStateConstants.HEADING_SWITCH_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_SWITCH_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.SWITCH_FRONT, armState, false).start();
//					}
//					// }
//				}
//			}
//		} else if (oi.operator.flipAndIntakeLow()) {
//			if (isOpen) {
//				new SetIntakePistons(false, true).start();
//			}
//			if (lift.isCarriageAtBottom()) {
//				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
//						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
//					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
//				}
//				if (armIsFront) {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR) {
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
//						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR, armState, false).start();
//					}
//
//				} else {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_INTAKE_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT, armState, false).start();
//					}
//				}
//			} else {
//				if (!isRunnigWithFlip) {
//					isRunnigWithFlip = true;
//					if (armIsFront) {
//						new DownAndFlipWhenPossibleIntakeRear().start();
//					} else {
//						new DownAndFlipWhenPossibleIntakeFront().start();
//					}
//				}
//			}
//
//		} else if (oi.operator.flipAndIntakeHigh()) {
//			if (isOpen) {
//				new SetIntakePistons(false, true).start();
//			}
//			if (lift.isCarriageAtBottom()) {
//				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2
//						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_2) {
//					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_2;
//					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE, liftState, 0).start();
//				}
//				if (armIsFront) {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_REAR;
//
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
//					}
//
//				} else {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_EXCHANGE_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.EXCHANGE_REAR, armState, false).start();
//					}
//
//				}
//			} else {
//				if (!isRunnigWithFlip) {
//					isRunnigWithFlip = true;
//
//					if (armIsFront) {
//						new DownAndFlipWhenPossible2ndIntakeRear().start();
//					} else {
//						new DownAndFlipWhenPossible2ndIntakeFront().start();
//					}
//				}
//
//			}
//		} else if (oi.operator.flipAndStore()) {
//			if (isOpen) {
//				new SetIntakePistons(false, true).start();
//			}
//			if (lift.isCarriageAtBottom()) {
//				if (liftState != LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE
//						&& liftState != LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
//					liftState = LiftSubsystem.LiftStateConstants.HEADING_INTAKE_EXCHANGE_STORE_STATE;
//					new LiftMotionProfile(LiftSubsystem.LiftEncoderConstants.LOW_STATE, liftState, 0).start();
//				}
//				if (armIsFront) {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_REAR;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_REAR, armState, false).start();
//					}
//
//				} else {
//					if (armState != ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT) {
//						armState = ArmSubsystem.ArmStateConstants.HEADING_STORE_FRONT;
//						new ArmMotionProfile(ArmSubsystem.ArmEncoderConstants.STORE_FRONT, armState, false).start();
//					}
//
//				}
//			} else {
//				if (!isRunnigWithFlip) {
//					isRunnigWithFlip = true;
//					if (armIsFront) {
//						new DownAndFlipWhenPossibleStoreRear().start();
//					} else {
//						new DownAndFlipWhenPossibleStoreFront().start();
//					}
//				}
//
//			}
//		} else if (oi.operator.isManual()) {
//			if (enteredManualMode == false) {
//				enteredManualMode = true;
//			}
//			liftState = LiftSubsystem.LiftStateConstants.CUSTOM_STATE;
//			armState = ArmSubsystem.ArmStateConstants.CUSTOM;
//			if (lift.isSecondStageAtBottom() && lift.isCarriageAtBottom()) {
//				if (oi.operator.getOperatorY() < 0) {
//					lift.setLift(0);
//				} else {
//					lift.setLift(oi.operator.getOperatorY());
//				}
//			} else {
//				lift.setLift(oi.operator.getOperatorY());
//			}
//			customLiftPos = (int) lift.getRawLift();
//			customArmPos = (int) arm.getArmRaw();
//
//			double armJoy = oi.operator.getManualArm();
//			if (armJoy == 0) {
//				arm.setArm(armJoy);
//				if (time.get() > 0.3) {
//					arm.setArmBrake(true);
//				}
//			} else {
//				arm.setArm(armJoy / 3);
//				arm.setArmBrake(false);
//				time.reset();
//			}
//		} else {
//			if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_EXCHANGE_STORE_STATE) {
//				lift.setLift(0);
//			} else if (liftState == LiftSubsystem.LiftStateConstants.SWITCH_STATE) {
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.SWITCH_STATE);
//			} else if (liftState == LiftSubsystem.LiftStateConstants.LOW_SCALE_STATE) {
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.LOW_SCALE_STATE);
//			} else if (liftState == LiftSubsystem.LiftStateConstants.MID_SCALE_STATE) {
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.MID_SCALE_STATE);
//			} else if (liftState == LiftSubsystem.LiftStateConstants.HIGH_SCALE_STATE) {
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.HIGH_SCALE_STATE);
//			} else if (liftState == LiftSubsystem.LiftStateConstants.CUSTOM_STATE) {
//				if (lift.getLiftState() != LiftSubsystem.LiftHalConstants.LOWEST_STATE) {
//					lift.setLift(0.15);
//				} else {
//					lift.setLift(0);
//				}
//			} else if (liftState == LiftSubsystem.LiftStateConstants.INTAKE_2) {
//				lift.setLiftMotion(LiftSubsystem.LiftEncoderConstants.INTAKE_2_STATE);
//			}
//			if (!isArmPidRunning) {
//				arm.setArmBrake(true);
//				// if(armState == ArmSubsystem.)
//			}
//			if (armState == ArmSubsystem.ArmStateConstants.INTAKE_FRONT) {
//				arm.setArm(0);
//			} else if (armState == ArmSubsystem.ArmStateConstants.INTAKE_REAR) {
//				arm.setArm(0);
//			}
//
//		}
//
//		if (oi.operator.deployOnlyWheels()) {
//			// if(armState == ArmSubsystem.ArmStateConstants.SWITCH_FRONT||
//			// armState ==
//			// ArmSubsystem.ArmStateConstants.SWITCH_REAR){
//			// new RunIntakeWheels(-0.6);
//			// }else{
//			new RunIntakeWheels(-1).start();
//			// }
//		} else if (oi.operator.lowSpeedDeploy()) {
//			new RunIntakeWheels(-0.35).start();
//		} else if (oi.operator.openIntakeToggle() && oi.operator.runIntakeWithWheelsClosed()) {
//			new SetIntakePistons(true, false).start();
//			new RunIntakeWheels(1).start();
//
//		} else if (oi.operator.openIntakeToggle()) {
//			new SetIntakePistons(true, false).start();
//			new RunIntakeWheels(0).start();
//		} else if (oi.operator.runIntakeWithWheelsClosed()) {
//			new SetIntakePistons(false, false).start();
//
//			new RunIntakeWheels(1).start();
//			// if(!isOpen){
//			// new SetIntakePistons(false, false).start();
//			// }
//
//			// new IntakeWithWheelsAndClose().start();
//		} else if (isOpen == false && !oi.operator.runIntakeWithWheelsClosed()
//				&& !(oi.operator.openIntakeToggle() || oi.operator.openIntakeToggleBB())) {
//			new SetIntakePistons(false, true).start();
//			Robot.intake.setIntakeMotors(0, 0);
//		} else {
//			new SetIntakePistons(false, true).start();
//			Robot.intake.setIntakeMotors(0, 0);
//		}
//		if (!arm.getInfraredSensor()) {
//			setLEDs(6);
//		}
//		if (arm.getInfraredSensor()) {
//			setLEDs(5);
//		}
//		prevStateFlipArm = oi.operator.flipArm();
//		if (arm.getArmRaw() > (ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT
//				+ ArmSubsystem.ArmEncoderConstants.INTAKE_REAR) / 2) {
//			armIsFront = true;
//		} else {
//			armIsFront = false;
//
//		}
//		if (lift.getLiftState() == LiftSubsystem.LiftHalConstants.LOWEST_STATE) {
//			lift.resetLiftEncoder();
//		}
//
//		if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kForward)) {
//			SmartDashboard.putString("intake", "forw");
//		} else if (intake.intakeSol.get().equals(DoubleSolenoid.Value.kReverse)) {
//			SmartDashboard.putString("intake", "rev");
//		}
//		if (arm.getArmHalZeroFront()) {
//			arm.setEncoder(ArmSubsystem.ArmEncoderConstants.INTAKE_FRONT);
//		} else if (arm.getArmHalZeroBack()) {
//			arm.setEncoder(ArmSubsystem.ArmEncoderConstants.INTAKE_REAR);
//		}
//		if (lift.isSecondStageAtBottom()) {
//			lidarOffset = lidar.getSample();
//		}
//		if (lidarCount == 10) {
//			lidarValue = lidar.getSample() - lidarOffset;
//			lidarCount = 0;
//		}
//		lidarCount++;
//		SmartDashboard.putNumber("adj Lidar", lidarValue);
//		SmartDashboard.putBoolean("Can Flip", lift.canFlip());
//		SmartDashboard.putNumber("Arm Current", arm.bottomMotor.getOutputCurrent());
//		SmartDashboard.putBoolean("ir", arm.getInfraredSensor());
//		
//		//***************************************************************** Prev States
//		prevStateIntakeToggle = oi.operator.openIntakeToggleBB();
//		prevStateIntakeToggle2 = oi.operator.openIntakeToggle();
//		prevStateFlipAndIntakeHigh = oi.operator.flipAndIntakeHigh();
//		prevStateFlipAndIntakeLow = oi.operator.flipAndIntakeLow();
//		prevStateFlipAndStore = oi.operator.flipAndStore();
//		//*****************************************************************************
//	}
//}
