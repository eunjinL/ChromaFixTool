using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

namespace Wind3_ImageTestTool
{
    public class ImageProcessingService
    {
        /// <summary>
        /// 이미지를 3개 영역으로 분할합니다
        /// </summary>
        /// <param name="image">원본 이미지</param>
        /// <returns>3개로 분할된 이미지 배열 (Top, Side, Bottom)</returns>
        public Bitmap[] SplitImageIntoThreeRegions(Bitmap image)
        {
            if (image == null)
                throw new ArgumentNullException(nameof(image));

            int regionWidth = image.Width / 3;
            var regions = new Bitmap[3];

            for (int i = 0; i < 3; i++)
            {
                // 인덱싱된 픽셀 형식 문제를 방지하기 위해 항상 24비트 RGB로 생성
                regions[i] = new Bitmap(regionWidth, image.Height, PixelFormat.Format24bppRgb);

                using (Graphics g = Graphics.FromImage(regions[i]))
                {
                    // 고품질 렌더링 설정
                    g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                    g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                    
                    // 각 영역을 복사 (Left=Top, Center=Side, Right=Bottom)
                    var sourceRect = new Rectangle(i * regionWidth, 0, regionWidth, image.Height);
                    var destRect = new Rectangle(0, 0, regionWidth, image.Height);

                    g.DrawImage(image, destRect, sourceRect, GraphicsUnit.Pixel);
                }
            }

            return regions;
        }

        /// <summary>
        /// RGB 채널을 분리합니다
        /// </summary>
        /// <param name="image">원본 이미지</param>
        /// <returns>R, G, B 채널 배열</returns>
        public Bitmap[] SplitChannels(Bitmap image)
        {
            if (image == null)
                throw new ArgumentNullException(nameof(image));

            var channels = new Bitmap[3]; // R, G, B
            for (int i = 0; i < 3; i++)
            {
                channels[i] = new Bitmap(image.Width, image.Height, PixelFormat.Format24bppRgb);
            }

            var bitmapData = image.LockBits(
                new Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);

            var rData = channels[0].LockBits(
                new Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            var gData = channels[1].LockBits(
                new Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
            var bData = channels[2].LockBits(
                new Rectangle(0, 0, image.Width, image.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            unsafe
            {
                byte* srcPtr = (byte*)bitmapData.Scan0;
                byte* rPtr = (byte*)rData.Scan0;
                byte* gPtr = (byte*)gData.Scan0;
                byte* bPtr = (byte*)bData.Scan0;

                int stride = bitmapData.Stride;

                for (int y = 0; y < image.Height; y++)
                {
                    for (int x = 0; x < image.Width; x++)
                    {
                        int offset = y * stride + x * 3;

                        byte b = srcPtr[offset];     // B
                        byte g = srcPtr[offset + 1]; // G  
                        byte r = srcPtr[offset + 2]; // R

                        // R 채널 (빨간색만)
                        rPtr[offset] = 0;     // B = 0
                        rPtr[offset + 1] = 0; // G = 0
                        rPtr[offset + 2] = r; // R

                        // G 채널 (초록색만)
                        gPtr[offset] = 0;     // B = 0
                        gPtr[offset + 1] = g; // G
                        gPtr[offset + 2] = 0; // R = 0

                        // B 채널 (파란색만)
                        bPtr[offset] = b;     // B
                        bPtr[offset + 1] = 0; // G = 0
                        bPtr[offset + 2] = 0; // R = 0
                    }
                }
            }

            image.UnlockBits(bitmapData);
            channels[0].UnlockBits(rData);
            channels[1].UnlockBits(gData);
            channels[2].UnlockBits(bData);

            return channels;
        }

        /// <summary>
        /// Bitmap을 WPF BitmapSource로 변환합니다
        /// </summary>
        /// <param name="bitmap">변환할 Bitmap</param>
        /// <returns>WPF BitmapSource</returns>
        public BitmapSource MatToImageSource(Bitmap bitmap)
        {
            if (bitmap == null)
                return null;

            try
            {
                var hBitmap = bitmap.GetHbitmap();
                var bitmapSource = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(
                    hBitmap, IntPtr.Zero, System.Windows.Int32Rect.Empty,
                    BitmapSizeOptions.FromEmptyOptions());

                // 메모리 해제
                DeleteObject(hBitmap);

                bitmapSource.Freeze(); // 성능 향상
                return bitmapSource;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Bitmap 변환 오류: {ex.Message}");
                return null;
            }
        }

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
                    g.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;
                    g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                    g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;

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

        // Win32 API for memory cleanup
        [System.Runtime.InteropServices.DllImport("gdi32.dll")]
        private static extern bool DeleteObject(IntPtr hObject);
    }
}
