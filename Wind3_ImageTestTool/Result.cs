using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wind3_ImageTestTool
{
    // 오프셋 계산 결과
    public struct OffsetResult
    {
        public PointF Offset { get; set; }
        public double NCC { get; set; }
        public string Method { get; set; }
        public float? ScaleX { get; }
        public float? ScaleY { get; }

        public OffsetResult(PointF offset, double ncc, string method, float? scaleX = null, float? scaleY = null)
        {
            Offset = offset;
            NCC = ncc;
            Method = method;
            ScaleX = scaleX;
            ScaleY = scaleY;
        }

        public OffsetResult WithScale((float scaleX, float scaleY) scale)
        {
            return new OffsetResult(Offset, NCC, Method, scale.scaleX, scale.scaleY);
        }
    }

    // 색수차 보정 결과 정보
    public struct ChromaticAberrationResult
    {
        public PointF RChannelOffset { get; set; }
        public PointF BChannelOffset { get; set; }
        public double RChannelNCC { get; set; }
        public double BChannelNCC { get; set; }
        public CorrectionMethod Method { get; set; }
        public BChanelGenerationMethod BChannelMethod { get; set; }
        public bool IsROIBased { get; set; }
        public string RegionInfo { get; set; }
        public TimeSpan ProcessingTime { get; set; }
    }

    public enum CorrectionMethod
    {
        NCC,    // Normalized Cross Correlation
        ORB     // ORB Feature Matching
    }

    public enum BChanelGenerationMethod
    {
        None = 0,      // 기존 B 채널 사용
        Average = 1,   // R과 G의 평균
        Weighted = 2,  // G에 높은 가중치 적용
        GreenOnly = 3   // G 채널만 사용
    }

    /// <summary>
    /// ROI 설정을 위한 구조체
    /// </summary>
    public struct ROI
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Width { get; set; }
        public int Height { get; set; }

        public ROI(int x, int y, int width, int height)
        {
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }

        public Rectangle ToRectangle() => new Rectangle(X, Y, Width, Height);
        public bool IsEmpty => Width <= 0 || Height <= 0;
    }
}
