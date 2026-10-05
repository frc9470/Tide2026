package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

/** Assign CAN and DIO ports here after the clone wiring is finalized. */
public final class Ports {
  private Ports() {}
  //random number for ports rn cuz idfk what their in
  public static final CanDeviceId TURRET_MOTOR = new CanDeviceId(0);
  
  public static final CanDeviceId RIGHT_ROLLER_MOTOR = new CanDeviceId(0);
  public static final CanDeviceId LEFT_ROLLER_MOTOR = new CanDeviceId(0);
  public static final CanDeviceId RIGHT_DEPLOY_MOTOR = new CanDeviceId(0);
  public static final CanDeviceId LEFT_DEPLOY_MOTOR = new CanDeviceId(0);
  public static final CanDeviceId KICKER_ROLLER = new CanDeviceId(0);
}
