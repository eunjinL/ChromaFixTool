using System;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace Wind3_ImageTestTool
{
    public class ImageProcessingService
    {
        /// <summary>
        /// ROI 영역 추출
        /// </summary>
        public Bitmap ExtractROI(Bitmap source, ROI roi)
        {
            if (source == null || roi.IsEmpty)
                return null;

            try
            {
                // ROI 영역이 이미지 범위를 벗어나지 않도록 조정
                int x = Math.Max(0, roi.X);
                int y = Math.Max(0, roi.Y);
                int width = Math.Min(source.Width - x, roi.Width);
                int height = Math.Min(source.Height - y, roi.Height);

                // ROI 영역 추출
                Bitmap roiImage = new Bitmap(width, height);
                using (Graphics g = Graphics.FromImage(roiImage))
                {
                    g.DrawImage(source, new Rectangle(0, 0, width, height),
                              new Rectangle(x, y, width, height), GraphicsUnit.Pixel);
                }

                return roiImage;
            }
            catch (Exception ex)
            {
                throw new Exception($"ROI 영역 추출 실패: {ex.Message}");
            }
        }
    }
} 