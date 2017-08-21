/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package armlab.lcm.msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class motion_command implements lcm.lcm.LCMEncodable
{
    public armlab.lcm.msgs.joint_value_quantity joint_position;
    public armlab.lcm.msgs.joint_value_quantity joint_velocity;
    public armlab.lcm.msgs.cartesian_pose cartesian_pose;
    public armlab.lcm.msgs.control_mode control_mode;
    public double timestamp;
 
    public motion_command()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xc0714d5769ac13acL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(armlab.lcm.msgs.motion_command.class))
            return 0L;
 
        classes.add(armlab.lcm.msgs.motion_command.class);
        long hash = LCM_FINGERPRINT_BASE
             + armlab.lcm.msgs.joint_value_quantity._hashRecursive(classes)
             + armlab.lcm.msgs.joint_value_quantity._hashRecursive(classes)
             + armlab.lcm.msgs.cartesian_pose._hashRecursive(classes)
             + armlab.lcm.msgs.control_mode._hashRecursive(classes)
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
 
        this.control_mode._encodeRecursive(outs); 
 
        outs.writeDouble(this.timestamp); 
 
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
 
    public static armlab.lcm.msgs.motion_command _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        armlab.lcm.msgs.motion_command o = new armlab.lcm.msgs.motion_command();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.joint_position = armlab.lcm.msgs.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.joint_velocity = armlab.lcm.msgs.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.cartesian_pose = armlab.lcm.msgs.cartesian_pose._decodeRecursiveFactory(ins);
 
        this.control_mode = armlab.lcm.msgs.control_mode._decodeRecursiveFactory(ins);
 
        this.timestamp = ins.readDouble();
 
    }
 
    public armlab.lcm.msgs.motion_command copy()
    {
        armlab.lcm.msgs.motion_command outobj = new armlab.lcm.msgs.motion_command();
        outobj.joint_position = this.joint_position.copy();
 
        outobj.joint_velocity = this.joint_velocity.copy();
 
        outobj.cartesian_pose = this.cartesian_pose.copy();
 
        outobj.control_mode = this.control_mode.copy();
 
        outobj.timestamp = this.timestamp;
 
        return outobj;
    }
 
}

