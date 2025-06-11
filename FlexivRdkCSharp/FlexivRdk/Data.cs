using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace FlexivRdkCSharp.FlexivRdk
{
    public static class FlexivConstants
    {
        public const int kCartDoF = 6;
        public const int kSerialJointDoF = 7;
        public const int kPoseSize = 7;
        public const int kIOPorts = 18;
        public const int kMaxExtAxes = 6;
    }

    public enum FlexivDataType
    {
        Int, Double, String, JPos, Coord, VectorInt, VectorDouble,
        VectorString, VectorJPos, VectorCoord
    }

    public class FlexivData
    {
        public FlexivDataType Type { get; set; }
        public object Value { get; set; }
        public FlexivData() { }
        public FlexivData(FlexivDataType type, object value)
        {
            Type = type;
            Value = value;
        }
        public static FlexivData Create<T>(T value) where T : notnull
        {
            var type = MapType(typeof(T));
            return new FlexivData(type, value);
        }
        public T As<T>()
        {
            if (Value is T typedValue && MapType(typeof(T)) == Type)
                return typedValue;
            throw new InvalidCastException($"Cannot convert {Type} to {typeof(T).Name}");
        }
        private static readonly Dictionary<Type, FlexivDataType> TypeMap = new()
        {
            { typeof(int), FlexivDataType.Int },
            { typeof(double), FlexivDataType.Double },
            { typeof(string), FlexivDataType.String },
            { typeof(JPos), FlexivDataType.JPos },
            { typeof(Coord), FlexivDataType.Coord },
            { typeof(List<int>), FlexivDataType.VectorInt },
            { typeof(List<double>), FlexivDataType.VectorDouble },
            { typeof(List<string>), FlexivDataType.VectorString },
            { typeof(List<JPos>), FlexivDataType.VectorJPos },
            { typeof(List<Coord>), FlexivDataType.VectorCoord }
        };
        private static FlexivDataType MapType(Type t)
        {
            if (TypeMap.TryGetValue(t, out var result))
                return result;
            throw new NotSupportedException($"Unsupported type: {t.FullName}");
        }

        public static implicit operator FlexivData(int value) => Create(value);
        public static implicit operator FlexivData(double value) => Create(value);
        public static implicit operator FlexivData(string value) => Create(value);
        public static implicit operator FlexivData(JPos value) => Create(value);
        public static implicit operator FlexivData(Coord value) => Create(value);
        public static implicit operator FlexivData(List<int> value) => Create(value);
        public static implicit operator FlexivData(List<double> value) => Create(value);
        public static implicit operator FlexivData(List<string> value) => Create(value);
        public static implicit operator FlexivData(List<JPos> value) => Create(value);
        public static implicit operator FlexivData(List<Coord> value) => Create(value);

        public static explicit operator int(FlexivData d) => d.As<int>();
        public static explicit operator double(FlexivData d) => d.As<double>();
        public static explicit operator string(FlexivData d) => d.As<string>();
        public static explicit operator JPos(FlexivData d) => d.As<JPos>();
        public static explicit operator Coord(FlexivData d) => d.As<Coord>();
        public static explicit operator List<int>(FlexivData d) => d.As<List<int>>();
        public static explicit operator List<double>(FlexivData d) => d.As<List<double>>();
        public static explicit operator List<string>(FlexivData d) => d.As<List<string>>();
        public static explicit operator List<JPos>(FlexivData d) => d.As<List<JPos>>();
        public static explicit operator List<Coord>(FlexivData d) => d.As<List<Coord>>();

        public override string ToString()
        {
            string valueStr;
            switch (Type)
            {
                case FlexivDataType.VectorInt:
                    valueStr = string.Join(", ", As<List<int>>());
                    break;
                case FlexivDataType.VectorDouble:
                    valueStr = string.Join(", ", As<List<double>>().Select(v => v.ToString("F3")));
                    break;
                case FlexivDataType.VectorString:
                    valueStr = string.Join(", ", As<List<string>>());
                    break;
                case FlexivDataType.VectorJPos:
                    valueStr = string.Join("\n  ", As<List<JPos>>());
                    break;
                case FlexivDataType.VectorCoord:
                    valueStr = string.Join("\n  ", As<List<Coord>>());
                    break;
                default:
                    valueStr = Value?.ToString() ?? "null";
                    break;
            }
            return $"({Type}) {valueStr}";
        }

    }

    public static class FlexivDataUtils
    {
        public static T Get<T>(Dictionary<string, FlexivData> dict, string key)
        {
            if (!dict.TryGetValue(key, out var data))
                throw new KeyNotFoundException($"Parameter '{key}' is missing.");
            try
            {
                return data.As<T>();
            }
            catch (InvalidCastException e)
            {
                throw new InvalidCastException($"Parameter '{key}' expected type {typeof(T).Name}, but got {data.Type}.", e);
            }
        }

        public static bool TryGet<T>(Dictionary<string, FlexivData> dict, string key, out T result)
        {
            if (dict.TryGetValue(key, out var data))
            {
                try
                {
                    result = data.As<T>();
                    return true;
                }
                catch
                { }
            }
            result = default!;
            return false;
        }
    }

    public class FlexivDataJsonConverter : JsonConverter<FlexivData>
    {
        public override FlexivData Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options)
        {
            using var doc = JsonDocument.ParseValue(ref reader);
            var root = doc.RootElement;
            var typeStr = root.GetProperty("type").GetString();
            if (!Enum.TryParse<FlexivDataType>(typeStr, out var type))
                throw new JsonException($"Unknown FlexivDataType: {typeStr}");
            var valueElem = root.GetProperty("value");
            object value = type switch
            {
                FlexivDataType.Int => valueElem.GetInt32(),
                FlexivDataType.Double => valueElem.GetDouble(),
                FlexivDataType.String => valueElem.GetString()!,
                FlexivDataType.JPos => JsonSerializer.Deserialize<JPos>(valueElem.GetRawText(), options)!,
                FlexivDataType.Coord => JsonSerializer.Deserialize<Coord>(valueElem.GetRawText(), options)!,
                FlexivDataType.VectorInt => JsonSerializer.Deserialize<List<int>>(valueElem.GetRawText(), options)!,
                FlexivDataType.VectorDouble => JsonSerializer.Deserialize<List<double>>(valueElem.GetRawText(), options)!,
                FlexivDataType.VectorString => JsonSerializer.Deserialize<List<string>>(valueElem.GetRawText(), options)!,
                FlexivDataType.VectorJPos => JsonSerializer.Deserialize<List<JPos>>(valueElem.GetRawText(), options)!,
                FlexivDataType.VectorCoord => JsonSerializer.Deserialize<List<Coord>>(valueElem.GetRawText(), options)!,
                _ => throw new JsonException($"Unsupported type: {type}")
            };
            return new FlexivData(type, value);
        }

        public override void Write(Utf8JsonWriter writer, FlexivData value, JsonSerializerOptions options)
        {
            writer.WriteStartObject();
            writer.WriteString("type", value.Type.ToString());
            writer.WritePropertyName("value");
            JsonSerializer.Serialize(writer, value.Value, options);
            writer.WriteEndObject();
        }
    }

    public class Coord
    {
        [JsonPropertyName("position")]
        public double[] Position { get; set; } = new double[FlexivConstants.kCartDoF / 2];
        [JsonPropertyName("orientation")]
        public double[] Orientation { get; set; } = new double[FlexivConstants.kCartDoF / 2];
        [JsonPropertyName("ref_frame")]
        public string[] RefFrame { get; set; } = new string[2] { "WORLD", "WORLD_ORIGIN" };
        [JsonPropertyName("ref_q")]
        public double[] RefQ { get; set; } = new double[FlexivConstants.kSerialJointDoF];
        [JsonPropertyName("ref_q_e")]
        public double[] RefQE { get; set; } = new double[FlexivConstants.kMaxExtAxes];
        public Coord() { }
        public Coord(double[] position, double[] orientation, string[] refFrame,
                     double[] refQ = null, double[] refQE = null)
        {
            if (position != null && position.Length != FlexivConstants.kCartDoF / 2)
                throw new ArgumentException($"Position must have length {FlexivConstants.kCartDoF / 2}", nameof(position));
            if (orientation != null && orientation.Length != FlexivConstants.kCartDoF / 2)
                throw new ArgumentException($"Orientation must have length {FlexivConstants.kCartDoF / 2}", nameof(orientation));
            if (refFrame != null && refFrame.Length != 2)
                throw new ArgumentException("RefFrame must have length 2", nameof(refFrame));
            if (refQ != null && refQ.Length != FlexivConstants.kSerialJointDoF)
                throw new ArgumentException($"RefQ must have length {FlexivConstants.kSerialJointDoF}", nameof(refQ));
            if (refQE != null && refQE.Length != FlexivConstants.kMaxExtAxes)
                throw new ArgumentException($"RefQE must have length {FlexivConstants.kMaxExtAxes}", nameof(refQE));
            Position = position ?? new double[FlexivConstants.kCartDoF / 2];
            Orientation = orientation ?? new double[FlexivConstants.kCartDoF / 2];
            RefFrame = refFrame ?? new string[2] { "WORLD", "WORLD_ORIGIN" };
            RefQ = refQ ?? new double[FlexivConstants.kSerialJointDoF];
            RefQE = refQE ?? new double[FlexivConstants.kMaxExtAxes];
        }
        public Coord(double x, double y, double z, double rx, double ry, double rz,
                     string coordType = "WORLD", string coordName = "WORLD_ORIGIN",
                     double[] refQ = null, double[] refQE = null)
        {
            if (refQ != null && refQ.Length != FlexivConstants.kSerialJointDoF)
                throw new ArgumentException($"RefQ must have length {FlexivConstants.kSerialJointDoF}", nameof(refQ));
            if (refQE != null && refQE.Length != FlexivConstants.kMaxExtAxes)
                throw new ArgumentException($"RefQE must have length {FlexivConstants.kMaxExtAxes}", nameof(refQE));
            Position = new double[] { x, y, z };
            Orientation = new double[] { rx, ry, rz };
            RefFrame = new string[] { coordType, coordName };
            RefQ = refQ ?? new double[FlexivConstants.kSerialJointDoF];
            RefQE = refQE ?? new double[FlexivConstants.kMaxExtAxes];
        }
        public Coord(double x, double y, double z, double qw, double qx, double qy, double qz,
             string coordType = "WORLD", string coordName = "WORLD_ORIGIN",
             double[] refQ = null, double[] refQE = null)
        {
            if (refQ != null && refQ.Length != FlexivConstants.kSerialJointDoF)
                throw new ArgumentException($"RefQ must have length {FlexivConstants.kSerialJointDoF}", nameof(refQ));
            if (refQE != null && refQE.Length != FlexivConstants.kMaxExtAxes)
                throw new ArgumentException($"RefQE must have length {FlexivConstants.kMaxExtAxes}", nameof(refQE));
            Position = new double[] { x, y, z };
            double yaw = 0, pitch = 0, roll = 0;
            Utility.Quat2EulerZYX(qw, qx, qy, qz, ref yaw, ref pitch, ref roll);
            Orientation = new double[] { Utility.Rad2Deg(roll), Utility.Rad2Deg(pitch), Utility.Rad2Deg(yaw) };
            RefFrame = new string[] { coordType, coordName };
            RefQ = refQ ?? new double[FlexivConstants.kSerialJointDoF];
            RefQE = refQE ?? new double[FlexivConstants.kMaxExtAxes];
        }

        public static Coord FromJson(string json)
        {
            return JsonSerializer.Deserialize<Coord>(json);
        }
        public override string ToString()
        {
            string FormatArray(double[] arr, int decimals = 5)
            {
                if (arr == null) return "null";
                string formatStr = $"F{decimals}";
                return string.Join(", ", arr.Select(v => v.ToString(formatStr)));
            }
            string pos = FormatArray(Position);
            string ori = FormatArray(Orientation);
            string refQ = FormatArray(RefQ);
            string refQE = FormatArray(RefQE);
            return $"Coord {{ Position: [{pos}], Orientation: [{ori}], RefFrame: \"{RefFrame}\", RefQ: [{refQ}], RefQE: [{refQE}] }}";
        }

    }

    public class JPos
    {
        [JsonPropertyName("q")]
        public double[] Q { get; set; } = new double[FlexivConstants.kSerialJointDoF];
        [JsonPropertyName("q_e")]
        public double[] QE { get; set; } = new double[FlexivConstants.kMaxExtAxes];
        public JPos() { }
        public JPos(double[] q, double[] q_e = null)
        {
            if (q != null && q.Length != FlexivConstants.kSerialJointDoF)
                throw new ArgumentException($"Q must have {FlexivConstants.kSerialJointDoF} elements.");
            if (q_e != null && q_e.Length != FlexivConstants.kMaxExtAxes)
                throw new ArgumentException($"Qe must have at most {FlexivConstants.kMaxExtAxes} elements.");
            Q = q ?? new double[FlexivConstants.kSerialJointDoF];
            QE = q_e ?? new double[FlexivConstants.kMaxExtAxes];
        }
        public JPos(double j1, double j2, double j3, double j4, double j5, double j6, double j7,
                    double e1 = 0, double e2 = 0, double e3 = 0, double e4 = 0, double e5 = 0, double e6 = 0)
        {
            Q = new double[] { j1, j2, j3, j4, j5, j6, j7 };
            QE = new double[] { e1, e2, e3, e4, e5, e6 };
        }
        public static JPos FromJson(string json)
        {
            return JsonSerializer.Deserialize<JPos>(json);
        }
        public override string ToString()
        {
            string FormatArray(double[] arr, int decimals = 5)
            {
                if (arr == null) return "null";
                string formatStr = $"F{decimals}";
                return string.Join(", ", arr.Select(v => v.ToString(formatStr)));
            }
            var qStr = FormatArray(Q);
            var qeStr = FormatArray(QE);
            return $"JPos {{ Q: [{qStr}], QE: [{qeStr}] }}";
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct FlexivError
    {
        public int error_code;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string error_msg;
    }

    public enum RobotMode : int
    {
        UNKNOWN = 0,
        IDLE = 1,
        RT_JOINT_TORQUE = 2,
        RT_JOINT_IMPEDANCE = 3,
        NRT_JOINT_IMPEDANCE = 4,
        RT_JOINT_POSITION = 5,
        NRT_JOINT_POSITION = 6,
        NRT_PLAN_EXECUTION = 7,
        NRT_PRIMITIVE_EXECUTION = 8,
        RT_CARTESIAN_MOTION_FORCE = 9,
        NRT_CARTESIAN_MOTION_FORCE = 10,
        MODES_CNT = 11
    }

    public enum OperationalStatus : int
    {
        UNKNOWN = 0,
        READY = 1,
        BOOTING = 2,
        ESTOP_NOT_RELEASED = 3,
        NOT_ENABLED = 4,
        RELEASING_BRAKE = 5,
        MINOR_FAULT = 6,
        CRITICAL_FAULT = 7,
        IN_REDUCED_STATE = 8,
        IN_RECOVERY_STATE = 9,
        IN_MANUAL_MODE = 10,
        IN_AUTO_MODE = 11
    }

    public enum CoordType : int
    {
        WORLD = 0,
        TCP = 1,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RobotState
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] Q;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] Theta;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] DQ;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] DTheta;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] Tau;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] TauDes;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] TauDot;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] TauExt;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kMaxExtAxes)]
        public double[] QE;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kMaxExtAxes)]
        public double[] DQE;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kMaxExtAxes)]
        public double[] TauE;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kPoseSize)]
        public double[] TcpPose;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kPoseSize)]
        public double[] TcpPoseDes;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] TcpVel;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kPoseSize)]
        public double[] FlangePose;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] FtSensorRaw;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] ExtWrenchInTcp;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] ExtWrenchInWorld;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] ExtWrenchInTcpRaw;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] ExtWrenchInWorldRaw;

        public override string ToString()
        {
            var sb = new System.Text.StringBuilder();
            void AppendArray(string name, double[] arr, int decimals = 5)
            {
                if (arr == null || arr.Length == 0)
                {
                    sb.AppendLine($"{name}: <empty>");
                    return;
                }
                string formatStr = $"F{decimals}";
                var formattedValues = arr.Select(v => v.ToString(formatStr));
                sb.AppendLine($"{name}: {string.Join(", ", formattedValues)}");
            }
            AppendArray(nameof(Q), Q);
            AppendArray(nameof(Theta), Theta);
            AppendArray(nameof(DQ), DQ);
            AppendArray(nameof(DTheta), DTheta);
            AppendArray(nameof(Tau), Tau);
            AppendArray(nameof(TauDes), TauDes);
            AppendArray(nameof(TauDot), TauDot);
            AppendArray(nameof(TauExt), TauExt);
            AppendArray(nameof(QE), QE);
            AppendArray(nameof(DQE), DQE);
            AppendArray(nameof(TauE), TauE);
            AppendArray(nameof(TcpPose), TcpPose);
            AppendArray(nameof(TcpPoseDes), TcpPoseDes);
            AppendArray(nameof(TcpVel), TcpVel);
            AppendArray(nameof(FlangePose), FlangePose);
            AppendArray(nameof(FtSensorRaw), FtSensorRaw);
            AppendArray(nameof(ExtWrenchInTcp), ExtWrenchInTcp);
            AppendArray(nameof(ExtWrenchInWorld), ExtWrenchInWorld);
            AppendArray(nameof(ExtWrenchInTcpRaw), ExtWrenchInTcpRaw);
            AppendArray(nameof(ExtWrenchInWorldRaw), ExtWrenchInWorldRaw);
            return sb.ToString();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PlanInfo
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string PTName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string NodeName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 512)]
        public string NodePath;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string NodePathTimePeriod;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string NodePathNumber;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string AssignedPlanName;
        public double VelocityScale;
        public int WaitingForStep;

        public override string ToString()
        {
            return $"  pt_name: {PTName}\n" +
                   $"  node_name: {NodeName}\n" +
                   $"  node_path: {NodePath}\n" +
                   $"  node_path_time_period: {NodePathTimePeriod}\n" +
                   $"  node_path_number: {NodePathNumber}\n" +
                   $"  assigned_plan_name: {AssignedPlanName}\n" +
                   $"  velocity_scale: {VelocityScale}\n" +
                   $"  waiting_for_step: {WaitingForStep}";
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RobotInfo
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string SerialNum;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string SoftwareVer;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string ModelName;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string LicenseType;
        public int DoF;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kCartDoF)]
        public double[] KxNom;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] KqNom;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] QMin;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] QMax;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] DqMax;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kSerialJointDoF)]
        public double[] TauMax;

        public override string ToString()
        {
            var sb = new StringBuilder();
            string FormatArray(double[] arr, int decimals = 5)
            {
                if (arr == null || arr.Length == 0)
                    return "<empty>";
                string formatStr = $"F{decimals}";
                return string.Join(", ", arr.Select(v => v.ToString(formatStr)));
            }
            sb.AppendLine($"SerialNum: {SerialNum}");
            sb.AppendLine($"SoftwareVer: {SoftwareVer}");
            sb.AppendLine($"ModelName: {ModelName}");
            sb.AppendLine($"LicenseType: {LicenseType}");
            sb.AppendLine($"DoF: {DoF}");
            sb.AppendLine($"KxNom: {FormatArray(KxNom)}");
            sb.AppendLine($"KqNom: {FormatArray(KqNom)}");
            sb.AppendLine($"QMin: {FormatArray(QMin)}");
            sb.AppendLine($"QMax: {FormatArray(QMax)}");
            sb.AppendLine($"DqMax: {FormatArray(DqMax)}");
            sb.AppendLine($"TauMax: {FormatArray(TauMax)}");
            return sb.ToString();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct ToolParams
    {
        public double Mass;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] CoM;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public double[] Inertia;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = FlexivConstants.kPoseSize)]
        public double[] TcpLocation;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GripperStates
    {
        public double Width;
        public double Force;
        public double MaxWidth;
    }
}
