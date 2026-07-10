using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Diagnostics;

namespace ArmTest
{
    public class ArmAPI
    {

        // 机械臂型号， RM_API_Init参数
        const int ARM_65 = 65;
        const int ARM_63_1 = 631;
        const int ARM_63_2 = 632;
        const int ARM_ECO65 = 651;
        const int ARM_75 = 75;
        const int ARM_ECO62 = 62;
        const int ARM_GEN72 = 72;

        const int RMERR_SUCC = 0;

        const int ARM_DOF = 7;

        public enum ARM_CTRL_MODES
        {
            None_Mode = 0,     //无规划
            Joint_Mode = 1,    //关节空间规划
            Line_Mode = 2,     //笛卡尔空间直线规划
            Circle_Mode = 3,   //笛卡尔空间圆弧规划
            Replay_Mode = 4,    //拖动示教轨迹复现
            Moves_Mode = 5      //样条曲线运动
        }

        //uint m_sockhandler;

        //        [StructLayout(LayoutKind.Sequential)]
        public enum POS_TEACH_MODES
        {
            X_Dir = 0,        //X轴方向
            Y_Dir = 1,       //Y轴方向
            Z_Dir = 2       //Z轴方向
        }

        ///       [StructLayout(LayoutKind.Sequential)]
        public enum ORT_TEACH_MODES
        {
            RX_Rotate = 0,       //RX轴方向
            RY_Rotate = 1,      //RY轴方向
            RZ_Rotate = 2      //RZ轴方向

        }

        public enum SensorType
        {
            B,
            ZF,
            SF
        }

        public enum RobotType
        {
            RM65,
            RM75,
            RML63I,
            RML63II,
            RML63III,
            ECO65,
            ECO62,
            GEN72,
            UNIVERSAL
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct POSE2
        {
            //位置
            public float px;
            public float py;
            public float pz;
            //四元数
            public float w;
            public float x;
            public float Y;
            public float z;
        }
        [StructLayout(LayoutKind.Sequential)]
        public struct ORT
        {
            public float rx;
            public float ry;
            public float rz;
        }
        [StructLayout(LayoutKind.Sequential)]
        public struct POSE
        {
            //位置
            public float px;
            public float py;
            public float pz;
            //欧拉角
            public float rx;
            public float ry;
            public float rz;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Pos
        {
            public float x; //* unit: m
            public float y;
            public float z;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Quat
        {
            public float w; //* unit: rad
            public float x;
            public float y;
            public float z;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Euler
        {
            public float rx; //* unit: rad
            public float ry;
            public float rz;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct Pose
        {
            public Pos position;
            public Quat quaternion;
            public Euler euler;
        }


        [StructLayout(LayoutKind.Sequential)]
        public struct KINEMATIC
        {
            public POSE2 pose;
            public ORT ort;
        }

        [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
        public struct FRAME_NAME
        {
            //位置
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public char[] name;
        }



        [StructLayout(LayoutKind.Sequential)]
        public struct FRAME
        {
            //位置
            public FRAME_NAME frame_name;
            public Pose pose;
            public float payload;
            public float x;
            public float y;
            public float z;
        }
        [StructLayout(LayoutKind.Sequential)]
        public struct JOINT_STATE
        {
            //位置
            /*[MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public float[] joint;*/
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public float[] temperature;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public float[] voltage;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public float[] current;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public byte[] en_state;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public UInt16[] err_flag;
            public UInt16 sys_err;
        }




        // 不使用如下修饰，会导致C#在调用完后，释放pData内容，导致C程序崩溃；所以在声明代理的时候，说明是C回调，不会收里面资源 
        [System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute(System.Runtime.InteropServices.CallingConvention.Cdecl)]
        public delegate void CallbackDelegate(int handler, int nKey, [MarshalAs(UnmanagedType.LPArray, SizeConst = 1024)] char[] sData, int len);
        public static CallbackDelegate callback;

        [DllImport("RM_Base.dll", EntryPoint = "RM_API_Init", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        private static extern int RM_API_Init(int devMode, CallbackDelegate cb);

        //初始化机械臂控制库
        [DllImport("RM_Base.dll", EntryPoint = "Arm_Socket_Start", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint Arm_Socket_Start([MarshalAs(UnmanagedType.LPStr)] string ip, int arm_prot, int recv_timeout);


        [DllImport("RM_Base.dll", EntryPoint = "Arm_Socket_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Arm_Socket_State(uint ArmSocket);

        [DllImport("RM_Base.dll", EntryPoint = "Arm_Socket_Close", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Arm_Socket_Close(uint ArmSocket);

        [DllImport("RM_Base.dll", EntryPoint = "API_Version", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr API_Version();

        //[DllImport("RM_Base.dll", EntryPoint = "Arm_Socket_Buffer_Clear", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        // public static extern void Arm_Socket_Buffer_Clear(uint ArmSocket);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Speed(uint ArmSocket, byte joint_num, float speed, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Acc(uint ArmSocket, byte joint_num, float acc, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Min_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Min_Pos(uint ArmSocket, byte joint_num, float joint, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Max_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Max_Pos(uint ArmSocket, byte joint_num, float joint, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Drive_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Drive_Speed(uint ArmSocket, byte joint_num, float speed);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Drive_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Drive_Acc(uint ArmSocket, byte joint_num, float acc);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Drive_Min_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Drive_Min_Pos(uint ArmSocket, byte joint_num, float joint);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Drive_Max_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Drive_Max_Pos(uint ArmSocket, byte joint_num, float joint);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_EN_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_EN_State(uint ArmSocket, byte joint_num, bool state, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Zero_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Zero_Pos(uint ArmSocket, byte joint_num, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Err_Clear", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Err_Clear(uint ArmSocket, byte joint_num, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Auto_Set_Joint_Limit", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Auto_Set_Joint_Limit(uint ArmSocket, byte limit_mode);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Speed(uint ArmSocket, [In, Out] float[] speed);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Acc(uint ArmSocket, [In, Out] float[] acc);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Min_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Min_Pos(uint ArmSocket, [In, Out] float[] min_joint);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Max_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Max_Pos(uint ArmSocket, [In, Out] float[] max_joint);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Drive_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Drive_Speed(uint ArmSocket, [In, Out] float[] speed);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Drive_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Drive_Acc(uint ArmSocket, [In, Out] float[] acc);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Drive_Min_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Drive_Min_Pos(uint ArmSocket, [In, Out] float[] min_joint);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Drive_Max_Pos", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Drive_Max_Pos(uint ArmSocket, [In, Out] float[] max_joint);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_EN_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_EN_State(uint ArmSocket, [In, Out] byte[] state);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Err_Flag", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Err_Flag(uint ArmSocket, [In, Out] UInt16[] state, [In, Out] UInt16[] bstate);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Software_Version", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Software_Version(uint ArmSocket, [In, Out] float[] version);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Line_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Line_Speed(uint ArmSocket, float speed, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Line_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Line_Acc(uint ArmSocket, float acc, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Angular_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Angular_Speed(uint ArmSocket, float speed, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Angular_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Angular_Acc(uint ArmSocket, float acc, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Line_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Line_Speed(uint ArmSocket, ref float speed);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Line_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Line_Acc(uint ArmSocket, ref float acc);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Angular_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Angular_Speed(uint ArmSocket, ref float speed);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Angular_Acc", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Angular_Acc(uint ArmSocket, ref float acc);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Tip_Init", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Tip_Init(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Collision_Stage", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Collision_Stage(uint ArmSocket, int stage, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Collision_Stage", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Collision_Stage(uint ArmSocket, ref int stage);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Joint_Zero_Offset", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Joint_Zero_Offset(uint ArmSocket, [In, Out] float[] offset, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Tool_Software_Version", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Tool_Software_Version(uint ArmSocket, ref int version);

        [DllImport("RM_Base.dll", EntryPoint = "Auto_Set_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Auto_Set_Tool_Frame(uint ArmSocket, byte point_num, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Generate_Auto_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Generate_Auto_Tool_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, float payload, float x, float y, float z, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Manual_Set_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Manual_Set_Tool_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, Pose pose, float payload, float x, float y, float z, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Change_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Change_Tool_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Delete_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Delete_Tool_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Current_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Current_Tool_Frame(uint ArmSocket, ref FRAME tool);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Given_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Given_Tool_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, ref FRAME tool);

        [DllImport("RM_Base.dll", EntryPoint = "Get_All_Tool_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_All_Tool_Frame(uint ArmSocket, [In, Out] FRAME_NAME[] names, ref int len);


        [DllImport("RM_Base.dll", EntryPoint = "Auto_Set_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Auto_Set_Work_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, byte point_num, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Manual_Set_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Manual_Set_Work_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, Pose pose, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Change_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Change_Work_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Delete_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Delete_Work_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Current_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Current_Work_Frame(uint ArmSocket, ref FRAME frame);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Given_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Given_Work_Frame(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string name, ref Pose pose);


        [DllImport("RM_Base.dll", EntryPoint = "Get_All_Work_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_All_Work_Frame(uint ArmSocket, [In, Out] FRAME_NAME[] names);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Current_Arm_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Current_Arm_State(uint ArmSocket, [In, Out] float[] joint, ref Pose pose, ref UInt16 Arm_Err, ref UInt16 Sys_Err);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Current_Arm_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Temperature(uint ArmSocket, [In, Out] float[] temperature);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Current", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Current(uint ArmSocket, [In, Out] float[] current);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Voltage", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Voltage(uint ArmSocket, [In, Out] float[] voltage);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Degree", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Degree(uint ArmSocket, [In, Out] float[] joint);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_All_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_All_State(uint ArmSocket, ref JOINT_STATE joint_state);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Plan_Num", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Plan_Num(uint ArmSocket, ref int plan);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Init_Pose", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Init_Pose(uint ArmSocket, [In, Out] float[] target, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Init_Pose", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Init_Pose(uint ArmSocket, [In, Out] float[] joint);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Install_Pose", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Install_Pose(uint ArmSocket, float x, float y, float z, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Install_Pose", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Install_Pose(uint ArmSocket, ref float fx, ref float fy, ref float fz, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Movej_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movej_Cmd(uint ArmSocket, [In, Out] float[] joint, byte v, float r, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Movel_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movel_Cmd(uint ArmSocket, Pose pose, byte v, float r, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Moves_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Moves_Cmd(uint ArmSocket, Pose pose, byte v, float r, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Movec_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movec_Cmd(uint ArmSocket, Pose pose_via, Pose pose_to, byte v, float r, byte loop, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Movej_CANFD", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movej_CANFD(uint ArmSocket, [In, Out] float[] joint, bool follow, float expand);

        [DllImport("RM_Base.dll", EntryPoint = "Movep_CANFD", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movep_CANFD(uint ArmSocket, Pose pose, bool follow);

        [DllImport("RM_Base.dll", EntryPoint = "MoveRotate_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int MoveRotate_Cmd(uint ArmSocket, int rotateAxis, float rotateAngle, Pose choose_axis, byte v, float r, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "MoveCartesianTool_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int MoveCartesianTool_Cmd(uint ArmSocket, [In, Out] float Joint_Cur, float movelengthx, float movelengthy, float movelengthz, int m_dev, byte v, float r, int trajectory_connect, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Move_Stop_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Move_Stop_Cmd(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Move_Pause_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Move_Pause_Cmd(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Move_Continue_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Move_Continue_Cmd(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Clear_Current_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_Current_Trajectory(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Clear_All_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_All_Trajectory(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Movej_P_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Movej_P_Cmd(uint ArmSocket, Pose pose, byte v, float r, int trajectory_connect, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Joint_Teach_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Joint_Teach_Cmd(uint ArmSocket, byte num, byte direction, byte v, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Pos_Teach_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Pos_Teach_Cmd(uint ArmSocket, POS_TEACH_MODES type, byte direction, byte v, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Ort_Teach_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Ort_Teach_Cmd(uint ArmSocket, ORT_TEACH_MODES type, byte direction, byte v, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Teach_Stop_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Teach_Stop_Cmd(uint ArmSocket, bool block);
        [DllImport("RM_Base.dll", EntryPoint = "Joint_Step_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Joint_Step_Cmd(uint ArmSocket, byte num, float step, byte v, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Pos_Step_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Pos_Step_Cmd(uint ArmSocket, POS_TEACH_MODES type, float step, byte v, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Ort_Step_Cmd", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Ort_Step_Cmd(uint ArmSocket, ORT_TEACH_MODES type, float step, byte v, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Controller_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Controller_State(uint ArmSocket, ref float voltage, ref float current, ref float temperature, ref UInt16 sys_err);

        [DllImport("RM_Base.dll", EntryPoint = "Set_WiFi_AP_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_WiFi_AP_Data(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string wifi_name, [MarshalAs(UnmanagedType.LPStr)] string password);

        [DllImport("RM_Base.dll", EntryPoint = "Set_WiFI_STA_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_WiFI_STA_Data(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string router_name, [MarshalAs(UnmanagedType.LPStr)] string password);

        [DllImport("RM_Base.dll", EntryPoint = "Set_USB_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_USB_Data(uint ArmSocket, int baudrate);

        [DllImport("RM_Base.dll", EntryPoint = "Set_RS485", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_RS485(uint ArmSocket, int baudrate);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Arm_Power", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Arm_Power(uint ArmSocket, bool cmd, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Power_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Power_State(uint ArmSocket, ref int power);


        [DllImport("RM_Base.dll", EntryPoint = "Clear_System_Err", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_System_Err(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Arm_Software_Version", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Arm_Software_Version(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string plan_version, [MarshalAs(UnmanagedType.LPStr)] string ctrl_version
            , [MarshalAs(UnmanagedType.LPStr)] string kernal1, [MarshalAs(UnmanagedType.LPStr)] string kernal2, [MarshalAs(UnmanagedType.LPStr)] string product_version);

        [DllImport("RM_Base.dll", EntryPoint = "Get_System_Runtime", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_System_Runtime(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string state, ref int day, ref int hour, ref int min, ref int sec);

        [DllImport("RM_Base.dll", EntryPoint = "Clear_System_Runtime", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_System_Runtime(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Joint_Odom", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Joint_Odom(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string state, [In, Out] float[] odom);

        [DllImport("RM_Base.dll", EntryPoint = "Clear_Joint_Odom", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_Joint_Odom(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_High_Speed_Eth", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_High_Speed_Eth(uint ArmSocket, byte num, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_IO_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_IO_State(uint ArmSocket, int IO, byte num, bool state, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_IO_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_IO_State(uint ArmSocket, int IO, byte num, ref byte state, ref byte mode);

        [DllImport("RM_Base.dll", EntryPoint = "Get_IO_Input", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_IO_Input(uint ArmSocket, ref byte DI_state, ref float AI_voltage);

        [DllImport("RM_Base.dll", EntryPoint = "Get_IO_Output", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_IO_Output(uint ArmSocket, ref byte DO_state, ref float AO_voltage);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Tool_DO_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Tool_DO_State(uint ArmSocket, byte num, bool state, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Tool_IO_Mode", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Tool_IO_Mode(uint ArmSocket, byte num, bool state, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Tool_IO_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Tool_IO_State(uint ArmSocket, [In, Out] float[] IO_Mode, [In, Out] float[] IO_State);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Tool_Voltage", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Tool_Voltage(uint ArmSocket, byte type, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Tool_Voltage", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Tool_Voltage(uint ArmSocket, ref byte voltage);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Gripper_Route", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Gripper_Route(uint ArmSocket, int min_limit, int max_limit, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Gripper_Release", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Gripper_Release(uint ArmSocket, int speed, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Gripper_Release", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Gripper_Pick(uint ArmSocket, int speed, int force, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Gripper_Pick_On", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Gripper_Pick_On(uint ArmSocket, int speed, int force, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Gripper_Position", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Gripper_Position(uint ArmSocket, int position, bool block);



        [DllImport("RM_Base.dll", EntryPoint = "Start_Drag_Teach", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Start_Drag_Teach(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Stop_Drag_Teach", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Stop_Drag_Teach(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Run_Drag_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Run_Drag_Trajectory(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Pause_Drag_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Pause_Drag_Trajectory(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Continue_Drag_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Continue_Drag_Trajectory(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Continue_Drag_Trajectory", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Stop_Drag_Trajectory(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Drag_Trajectory_Origin", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Drag_Trajectory_Origin(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Start_Multi_Drag_Teach", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Start_Multi_Drag_Teach(uint ArmSocket, int mode, int singular_wall, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Force_Postion", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Force_Postion(uint ArmSocket, int sensor, int mode, int direction, int N, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Stop_Force_Postion", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Stop_Force_Postion(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Force_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Force_Data(uint ArmSocket, [In, Out] float[] Force, [In, Out] float[] zero_force);


        [DllImport("RM_Base.dll", EntryPoint = "Clear_Force_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_Force_Data(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Force_Sensor", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Force_Sensor(uint ArmSocket);


        [DllImport("RM_Base.dll", EntryPoint = "Manual_Set_Force", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Manual_Set_Force(uint ArmSocket, int type, [In, Out] float[] joint);


        [DllImport("RM_Base.dll", EntryPoint = "Stop_Set_Force_Sensor", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Stop_Set_Force_Sensor(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Hand_Posture", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Hand_Posture(uint ArmSocket, int posture_num, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Hand_Seq", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Hand_Seq(uint ArmSocket, int seq_num, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Hand_Angle", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Hand_Angle(uint ArmSocket, [In, Out] int[] angle, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Hand_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Hand_Speed(uint ArmSocket, int speed, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Hand_Force", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Hand_Force(uint ArmSocket, int force, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Fz", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Fz(uint ArmSocket, ref float Fz);


        [DllImport("RM_Base.dll", EntryPoint = "Clear_Fz", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Clear_Fz(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Auto_Set_Fz", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Auto_Set_Fz(uint ArmSocket);


        [DllImport("RM_Base.dll", EntryPoint = "Manual_Set_Fz", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Manual_Set_Fz(uint ArmSocket, [In, Out] float[] joint1, [In, Out] float joint2);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Modbus_Mode", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Modbus_Mode(uint ArmSocket, int port, int baudrate, int timeout, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Close_Modbus_Mode", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Close_Modbus_Mode(uint ArmSocket, int port, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Read_Coils", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Read_Coils(uint ArmSocket, int port, int address, int num, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Read_Multiple_Coils", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Read_Multiple_Coils(uint ArmSocket, int port, int address, int num, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Read_Input_Status", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Read_Input_Status(uint ArmSocket, int port, int address, int num, int device, ref int coils_data);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Read_Holding_Registers", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Read_Holding_Registers(uint ArmSocket, int port, int address, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Read_Multiple_Holding_Registers", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Read_Multiple_Holding_Registers(uint ArmSocket, int port, int address, int num, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Read_Input_Registers", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Read_Input_Registers(uint ArmSocket, int port, int address, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Read_Multiple_Input_Registers", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Read_Multiple_Input_Registers(uint ArmSocket, int port, int address, int num, int device, ref int coils_data);

        [DllImport("RM_Base.dll", EntryPoint = "Write_Single_Coil", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Write_Single_Coil(uint ArmSocket, int port, int address, int data, int device, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Write_Coils", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Write_Coils(uint ArmSocket, int port, int address, int num, [In, Out] byte[] coils_data, int device, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Write_Single_Register", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Write_Single_Register(uint ArmSocket, int port, int address, int data, int device, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Write_Registers", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Write_Registers(uint ArmSocket, int port, int address, int num, [In, Out] byte[] single_data, int device, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Set_Lift_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Lift_Speed(uint ArmSocket, int speed);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Lift_Height", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Lift_Height(uint ArmSocket, int height, int speed, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Get_Lift_State", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Lift_State(uint ArmSocket, ref int height, ref int current, ref int err_flag, ref int mode);


        [DllImport("RM_Base.dll", EntryPoint = "Start_Force_Position_Move", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Start_Force_Position_Move(uint ArmSocket, bool block);


        [DllImport("RM_Base.dll", EntryPoint = "Force_Position_Move_Pose", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Force_Position_Move_Pose(uint ArmSocket, Pose pose, byte sensor, byte mode, int dir, float force, bool follow);


        [DllImport("RM_Base.dll", EntryPoint = "Force_Position_Move_Joint", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Force_Position_Move_Joint(uint ArmSocket, ref float joint, byte sensor, byte mode, int dir, float force, bool follow);

        [DllImport("RM_Base.dll", EntryPoint = "Stop_Force_Position_Move", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Stop_Force_Position_Move(uint ArmSocket, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Teach_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Set_Teach_Frame(uint ArmSocket, int type, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Get_Teach_Frame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_Teach_Frame(uint ArmSocket, ref int type);

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct Send_Project_Params
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 300)] // 使用MarshalAs属性来确保字符串的固定大小  
            public string project_path;      // 下发文件路径文件名
            public int project_path_len;   // 名称长度
            public int plan_speed;     // 规划速度比例系数
            public int only_save;      // 0-运行文件，1-仅保存文件，不运行
            public int save_id;        // 保存到控制器中的编号
            public int step_flag;      // 设置单步运行方式模式，1-设置单步模式 0-设置正常运动模式
            public int auto_start;     // 设置默认在线编程文件，1-设置默认  0-设置非默认
        }

        [DllImport("RM_Base.dll", EntryPoint = "Send_TrajectoryFile", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Send_TrajectoryFile(uint ArmSocket, Send_Project_Params project, ref int errline);

        [DllImport("RM_Base.dll", EntryPoint = "Set_Plan_Speed", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern KINEMATIC Set_Plan_Speed(uint ArmSocket, int speed, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Popup", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Popup(uint ArmSocket, int content, bool block);

        [DllImport("RM_Base.dll", EntryPoint = "Set_High_Ethernet", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern KINEMATIC Set_High_Ethernet(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string ip, [MarshalAs(UnmanagedType.LPStr)] string mask, [MarshalAs(UnmanagedType.LPStr)] string gateway);

        [DllImport("RM_Base.dll", EntryPoint = "Get_High_Ethernet", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Get_High_Ethernet(uint ArmSocket, [MarshalAs(UnmanagedType.LPStr)] string ip, [MarshalAs(UnmanagedType.LPStr)] ref string mask, [MarshalAs(UnmanagedType.LPStr)]
        ref string gateway, [MarshalAs(UnmanagedType.LPStr)] ref string mac);

        [DllImport("RM_Base.dll", EntryPoint = "Save_Device_Info_All", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern KINEMATIC Save_Device_Info_All(uint ArmSocket);


        [DllImport("RM_Base.dll", EntryPoint = "Algo_Init_Sys_Data", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Init_Sys_Data(SensorType devMode, RobotType bType);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Set_Angle", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Set_Angle(float x, float y, float z);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Get_Angle", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Get_Angle(ref float x, ref float y, ref float z);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Forward_Kinematics", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern Pose Algo_Forward_Kinematics([In, Out] float[] joint);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Inverse_Kinematics", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern int Algo_Inverse_Kinematics([In, Out] float[] q_in, ref Pose q_pose, [In, Out] float[] q_out, byte flag);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Set_WorkFrame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Set_WorkFrame(FRAME coord_work);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Get_Curr_WorkFrame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Get_Curr_WorkFrame(ref FRAME coord_work);

        [DllImport("RM_Base.dll", EntryPoint = "Algo_Set_ToolFrame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Set_ToolFrame(FRAME coord_tool);


        [DllImport("RM_Base.dll", EntryPoint = "Algo_Get_Curr_ToolFrame", CharSet = CharSet.Auto, CallingConvention = CallingConvention.Cdecl)]
        public static extern void Algo_Get_Curr_ToolFrame(ref FRAME coord_tool);



        static uint sockhandle;
        // 基础接口调用
        static void Demo1()
        {
            //网络连接状态
            int ret = Arm_Socket_State(sockhandle);
            Console.WriteLine("state:{0}", ret);

            //清除关节错误代码
            ret = Set_Joint_Err_Clear(sockhandle, 1, true);
            Console.WriteLine("state:{0}", ret);

            //获取关节最大速度
            float[] fSpeeds = new float[7];
            ret = Get_Joint_Speed(sockhandle, fSpeeds);
            Console.WriteLine("Joint Accelerations: " + string.Join(", ", fSpeeds));

            //获取关节最大加速度
            float[] fAccs = new float[7];
            ret = Get_Joint_Acc(sockhandle, fAccs);
            Console.WriteLine("state:{0}", ret);

            //获取关节最小限位
            float[] fMinJoint = new float[7];
            ret = Get_Joint_Min_Pos(sockhandle, fMinJoint);
            Console.WriteLine("state:{0}", ret);

            //获取关节最大限位
            float[] fMaxJoint = new float[7];
            ret = Get_Joint_Max_Pos(sockhandle, fMaxJoint);
            Console.WriteLine("state:{0}", ret);


            FRAME tool = new FRAME();
            ret = Get_Current_Tool_Frame(sockhandle, ref tool);
            string arr = new string(tool.frame_name.name);
            Console.WriteLine("tool:{0},{1}", ret, arr);
        }

        // 运动接口调用
        static void Demo2()
        {
            // 回零位
            float[] fJoint = { 0, 0, 0, 0, 0, 0 };
            int ret = Movej_Cmd(sockhandle, fJoint, 20, 0, 0, true);
            Console.WriteLine("state:{0}", ret);

            // 动作1
            fJoint[1] = -20;
            fJoint[2] = -70;
            fJoint[4] = -90;
            ret = Movej_Cmd(sockhandle, fJoint, 20, 0, 0, true); ;
            Console.WriteLine("state:{0}", ret);
        }

        // 力位混合控制
        static void Demo3()
        {
            // 开启力控
            Start_Force_Position_Move(sockhandle, true);
            // 获取当前位姿
            Pose mpose;
            mpose.position.x = 0;
            mpose.position.y = 0;
            mpose.position.z = 0;
            mpose.euler.rx = 0;
            mpose.euler.ry = 0;
            mpose.euler.rz = 0;
            mpose.quaternion.w = 0;
            mpose.quaternion.x = 0;
            mpose.quaternion.y = 0;
            mpose.quaternion.z = 0;

            float[] fJoint = { 0, 0, 0, 0, 0, 0 };
            UInt16 sys_err = 0;
            UInt16 arm_err = 0;
            Get_Current_Arm_State(sockhandle, fJoint, ref mpose, ref arm_err, ref sys_err);
            for (int i = 0; i < 100; i++)
            {
                mpose.position.x += 0.001f;
                Force_Position_Move_Pose(sockhandle, mpose, 0, 0, 2, -5, false);
            }

            Stop_Force_Position_Move(sockhandle, true);
        }

        // 位姿示教
        static void Demo4()
        {
            float[] fJoint = { 0, 0, 0, 0, 0, 0 };
            // 动作1
            fJoint[1] = -35;
            fJoint[2] = -60;
            fJoint[4] = -60;
            int ret = Movej_Cmd(sockhandle, fJoint, 20, 0, 0, true);
            Console.WriteLine("初始位置：" + ret);
            ret = Ort_Teach_Cmd(sockhandle, ORT_TEACH_MODES.RX_Rotate, 1, 20, true);
            Console.WriteLine("示教状态：" + ret);
            Thread.Sleep(2000);
            ret = Teach_Stop_Cmd(sockhandle, true);
            Console.WriteLine("示教停止：" + ret);
        }
        public static Task<uint> ConnectArm(string ip)
        {
            return ConnectArm(ip, ARM_65);
        }

        public static async Task<uint> ConnectArm(string ip, int devMode, int port = 8080, int recvTimeoutMs = 3000)
        {
            if (devMode <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(devMode), devMode, "RealMan SDK devMode 必须由官方文档或厂家确认，不能使用无效值。不中断保护：不初始化 SDK。");
            }

            return await Task.Run(() =>
            {
                Debug.WriteLine($"this is RM-ROBOT! devMode={devMode}");
                //连接
                RM_API_Init(devMode, null);
                sockhandle = Arm_Socket_Start(ip, port, recvTimeoutMs);

                IntPtr versionPtr = API_Version();
                // 如果返回字符串，将 IntPtr 转换为字符串
                string version = Marshal.PtrToStringAnsi(versionPtr);
                // 打印结果
                Debug.WriteLine("API 版本：" + version);

                Debug.WriteLine("sockhandle：" + sockhandle);
                return sockhandle;
            });

            // 基础获取状态接口调用
            //Demo1();

            // 运动类型接口调用
            //Demo2();

            // 力控测试 - 在Demo2的基础上使用
            //Demo3();

            // 位姿示教
            //Demo4();

        }


    }
}
