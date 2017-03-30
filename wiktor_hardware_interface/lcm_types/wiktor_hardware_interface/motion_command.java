/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package wiktor_hardware_interface;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class motion_command implements lcm.lcm.LCMEncodable
{
    public wiktor_hardware_interface.joint_value_quantity joint_position;
    public wiktor_hardware_interface.joint_value_quantity joint_velocity;
    public wiktor_hardware_interface.cartesian_value_quantity cartesian_pose;
    public long utime;
    public byte command_type;
 
    public motion_command()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x7996715ba0ceabeaL;
 
    public static final byte IS_POSITION_MOTION = (byte) 0;
    public static final byte IS_VELOCITY_MOTION = (byte) 2;
    public static final byte IS_CARTESIAN_MOTION = (byte) 1;
    public static final byte JOINT_POSITION = (byte) 0;
    public static final byte JOINT_POSITION_VELOCITY = (byte) 2;
    public static final byte CARTESIAN_POSE = (byte) 1;

    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(wiktor_hardware_interface.motion_command.class))
            return 0L;
 
        classes.add(wiktor_hardware_interface.motion_command.class);
        long hash = LCM_FINGERPRINT_BASE
             + wiktor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + wiktor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + wiktor_hardware_interface.cartesian_value_quantity._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        this.joint_position._encodeRecursive(outs); 
 
        this.joint_velocity._encodeRecursive(outs); 
 
        this.cartesian_pose._encodeRecursive(outs); 
 
        outs.writeLong(this.utime); 
 
        outs.writeByte(this.command_type); 
 
    }
 
    public motion_command(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public motion_command(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static wiktor_hardware_interface.motion_command _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        wiktor_hardware_interface.motion_command o = new wiktor_hardware_interface.motion_command();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.joint_position = wiktor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.joint_velocity = wiktor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.cartesian_pose = wiktor_hardware_interface.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.utime = ins.readLong();
 
        this.command_type = ins.readByte();
 
    }
 
    public wiktor_hardware_interface.motion_command copy()
    {
        wiktor_hardware_interface.motion_command outobj = new wiktor_hardware_interface.motion_command();
        outobj.joint_position = this.joint_position.copy();
 
        outobj.joint_velocity = this.joint_velocity.copy();
 
        outobj.cartesian_pose = this.cartesian_pose.copy();
 
        outobj.utime = this.utime;
 
        outobj.command_type = this.command_type;
 
        return outobj;
    }
 
}

