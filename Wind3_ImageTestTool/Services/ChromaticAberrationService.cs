using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV.Features2D;
// System.Drawing 명시적 사용
using SystemSize = System.Drawing.Size;

namespace Wind3_ImageTestTool
{
    /// <summary>
    /// 색수차 보정 서비스 클래스
    /// </summary>
    public class ChromaticAberrationService
    {
        private readonly ImageProcessingService imageProcessingService;
        private ProcessingOptions optionParam;
        public string saveDir = @"D:\ImageLog";
        public string savePath_R = @"D:\ImageLog\RChannel";
        public string savePath_B = @"D:\ImageLog\BChannel";

        public ChromaticAberrationService()
        {
            imageProcessingService = new ImageProcessingService();
            optionParam = new ProcessingOptions();
        }

        public void SetOptions(ProcessingOptions options)
        {
            optionParam = options;
        }

        private Bitmap ScaleUpImage(Bitmap original, int scale)
        {
            Mat src = BitmapToMat(original);
            Mat resized = new Mat();
            CvInvoke.Resize(src, resized, new Size(src.Width * scale, src.Height * scale), interpolation: Inter.Cubic);

            // 결과를 Bitmap으로 변환
            using (var buffer = new Emgu.CV.Util.VectorOfByte())
            {
                CvInvoke.Imencode(".bmp", resized, buffer);
                using (var ms = new MemoryStream(buffer.ToArray()))
                {
                    return new Bitmap(ms);
                }
            }
        }

        /// <summary>
        /// 1. ROI 설정 및 색수차 보정 함수 호출
        /// </summary>
        /// <param name="originalImage">원본 이미지</param>
        /// <param name="method">보정 방식</param>
        /// <param name="bChannelMethod">B채널 생성 방식</param>
        /// <param name="regionROIs">ROI</param>
        /// <returns>보정된 이미지와 상세 정보</returns>
        public (Bitmap CorrectedImage, ChromaticAberrationResult[] RegionResults) CorrectChromaticAberrationWithROIDetails(
            Bitmap originalImage, CorrectionMethod method, BChanelGenerationMethod bChannelMethod, int imageIndex, ROI[] regionROIs = null)
        {
            // 1. ROI 설정
            ROI singleROI;
            if (regionROIs != null && regionROIs.Length > 0 && !regionROIs[0].IsEmpty)
            {
                singleROI = regionROIs[0];
                Debug.WriteLine("[ROI] 사용자 정의 ROI 사용");
            }
            else
            {
                int roiWidth = (int)(originalImage.Width * 0.4);
                int roiHeight = (int)(originalImage.Height * 0.4);
                int roiX = (originalImage.Width - roiWidth) / 2;
                int roiY = (originalImage.Height - roiHeight) / 2;
                singleROI = new ROI(roiX, roiY, roiWidth, roiHeight);
                Debug.WriteLine("[ROI] 기본 ROI 사용 - 전체 이미지 중앙 40% 영역");
            }
            Debug.WriteLine($"[ROI] 사용할 ROI: ({singleROI.X}, {singleROI.Y}, {singleROI.Width}, {singleROI.Height})");

            // 2. 프레임 범위 체크
            if (imageIndex < optionParam.StartFrameIndex || imageIndex > optionParam.EndFrameIndex)
            {
                Debug.WriteLine($"[Frame] {imageIndex}번 프레임은 처리 범위를 벗어나 원본 유지");
                return (originalImage, new ChromaticAberrationResult[] 
                { 
                    new ChromaticAberrationResult 
                    { 
                        Method = method,
                        BChannelMethod = bChannelMethod,
                        IsROIBased = true,
                        RegionInfo = $"처리 범위를 벗어난 프레임 (원본 유지)",
                        ProcessingTime = TimeSpan.Zero
                    } 
                });
            }

            // 3. ROI 기반 색수차 보정
            var regionResults = new ChromaticAberrationResult[1]; // 단일 결과
            try
            {
                // (1) 전체 이미지에 대해 ROI 기반 색수차 보정
                var regionStartTime = DateTime.Now;
                var (correctedImage, rResult, bResult) = CorrectRegionChromaticAberrationWithROI(
                    originalImage, method, bChannelMethod, singleROI, imageIndex);
                
                // (2) 결과 정보 저장
                regionResults[0] = new ChromaticAberrationResult
                {
                    RChannelOffset = rResult.Offset,
                    BChannelOffset = bResult.Offset,
                    RChannelNCC = rResult.NCC,
                    BChannelNCC = bResult.NCC,
                    Method = method,
                    BChannelMethod = bChannelMethod,
                    IsROIBased = true,
                    RegionInfo = $"전체 이미지 - ROI:({singleROI.X},{singleROI.Y},{singleROI.Width},{singleROI.Height}) {((regionROIs != null && regionROIs.Length > 0 && !regionROIs[0].IsEmpty) ? "(사용자 정의)" : "(기본값)")}",
                    ProcessingTime = DateTime.Now - regionStartTime
                };

                // (3) 원본 이미지 복사
                try
                {
                    string debugDir = Path.Combine(saveDir, "Original_Debug");
                    if (!Directory.Exists(debugDir))
                        Directory.CreateDirectory(debugDir);
                    SaveImage(originalImage, Path.Combine(debugDir, $"{imageIndex}_Original.png"));
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"[ERROR] 원본 이미지 저장 실패: {ex.Message}");
                }

                return (correctedImage, regionResults);
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"ROI 기반 색수차 보정 실패: {ex.Message}", ex);
            }
        }

        /// <summary>
        /// 2. R,G,B 분리 및 보정 이미지 생성
        /// </summary>
        /// <param name="regionImage">영역 이미지</param>
        /// <param name="method">보정 방식</param>
        /// <param name="bChannelMethod">B채널 생성 방식</param>
        /// <param name="roi">ROI 영역</param>
        /// <returns>보정된 영역 이미지와 채널별 오프셋 정보</returns>
        private (Bitmap CorrectedImage, OffsetResult RResult, OffsetResult BResult) CorrectRegionChromaticAberrationWithROI(
            Bitmap regionImage, CorrectionMethod method, BChanelGenerationMethod bChannelMethod, ROI roi, int imageIndex)
        {
            // 1. RGB 채널 분리
            var channels = imageProcessingService.SplitChannels(regionImage);
            var rChannel = ConvertToGrayscale(channels[0], "R"); // R 채널 (그레이스케일)
            var gChannel = ConvertToGrayscale(channels[1], "G"); // G 채널 (기준)
            var bChannel = ConvertToGrayscale(channels[2], "B"); // B 채널 (그레이스케일)

            try
            {
                OffsetResult rResult, bResult;
                // 2. ROI 기반 오프셋 계산 (g는 기준이고, r이랑 b에 대해서 진행)
                rResult = CalculateChannelOffsetWithROIDetails(imageIndex, gChannel, rChannel, roi, method, savePath_R);
                bResult = CalculateChannelOffsetWithROIDetails(imageIndex, gChannel, bChannel, roi, method, savePath_B);

                // 3. 채널 오프셋 적용하여 보정된 이미지 생성
                Bitmap correctedImage;
                if (bChannelMethod == BChanelGenerationMethod.None)
                {
                    // (1) 기존 방식
                    correctedImage = ApplyChromaticAberrationCorrection(regionImage, rResult, bResult, roi);
                }
                else
                {
                    // (2) B채널 생성 방식: R, B 먼저 보정 → 보정된 이미지에서 B채널 재생성
                    var tempCorrectedImage = ApplyChromaticAberrationCorrection(regionImage, rResult, bResult, roi);
                    correctedImage = RegenerateBChannelFromCorrected(tempCorrectedImage, bChannelMethod);
                    tempCorrectedImage.Dispose();
                }

                return (correctedImage, rResult, bResult);
            }
            finally
            {
                // 메모리 정리
                foreach (var channel in channels)
                    channel?.Dispose();
                rChannel?.Dispose();
                gChannel?.Dispose();
                bChannel?.Dispose();
            }
        }

        /// <summary>
        /// 3. ROI 별 실제 색수차 Offset 값 생성
        /// (1) reference - g채널
        /// (2) target - 각각 r과 b채널
        /// </summary>
        private OffsetResult CalculateChannelOffsetWithROIDetails(int imageIndex, Bitmap referenceChannel, Bitmap targetChannel, ROI roi, CorrectionMethod method, string saveDir)
        {
            try
            {
                // 1. ROI 영역 검증 및 조정 (이미지 밖으로 나오면 잘라냄)
                var roiRect = roi.ToRectangle();
                roiRect.Intersect(new Rectangle(0, 0, referenceChannel.Width, referenceChannel.Height));
                
                if (roiRect.Width < 20 || roiRect.Height < 20)
                {
                    Debug.WriteLine("ROI가 너무 작음, 기본 방식 적용");
                    var fallbackResult = new OffsetResult(new PointF(0, 0), 0.0, "ROI_TOO_SMALL");
                    return fallbackResult;
                }

                // 1-1. ROI 크기 확인 및 스케일 업 결정
                const int MIN_ROI_SIZE = 100;
                bool needScaleUp = roiRect.Width < MIN_ROI_SIZE || roiRect.Height < MIN_ROI_SIZE;
                Bitmap workingReference = referenceChannel;
                Bitmap workingTarget = targetChannel;
                Rectangle workingRoiRect = roiRect;
                int scaleFactor = 1;
                if (needScaleUp && optionParam.IsScaleUpEnabled)
                {
                    float scaleX = (float)MIN_ROI_SIZE / roiRect.Width;
                    float scaleY = (float)MIN_ROI_SIZE / roiRect.Height;
                    scaleFactor = (int)Math.Ceiling(Math.Max(scaleX, scaleY));
                    Debug.WriteLine($"[ROI] ROI 크기가 작아 스케일 업 적용 (원본: {roiRect.Width}x{roiRect.Height})");
                    workingReference = ScaleUpImage(referenceChannel, scaleFactor);
                    workingTarget = ScaleUpImage(targetChannel, scaleFactor);
                    workingRoiRect = new Rectangle(
                        roiRect.X * scaleFactor,
                        roiRect.Y * scaleFactor,
                        roiRect.Width * scaleFactor,
                        roiRect.Height * scaleFactor
                    );
                    Debug.WriteLine($"[ROI] 스케일 업 후 ROI: {workingRoiRect.Width}x{workingRoiRect.Height}");
                }

                // 2. NCC or ORB로 색수차 Offset 값 생성
                OffsetResult result;
                switch (method)
                {
                    case CorrectionMethod.NCC:
                        result = CalculateOffsetUsingNCCWithROIDetails(imageIndex, workingReference, workingTarget, workingRoiRect, saveDir); 
                        if (needScaleUp)
                        {
                            result = new OffsetResult(new PointF(result.Offset.X / scaleFactor, result.Offset.Y / scaleFactor), result.NCC, result.Method);
                        }
                        Debug.WriteLine($"[NCC] NCC 결과: ({result.Offset.X:F3}, {result.Offset.Y:F3}), NCC: {result.NCC:F4}, 방법: {result.Method}");
                        break;
                    case CorrectionMethod.ORB:
                        var orbOffset = CalculateOffsetUsingORBWithROI(imageIndex, workingReference, workingTarget, workingRoiRect, saveDir);
                        if (needScaleUp)
                        {
                            orbOffset = new PointF(orbOffset.X / scaleFactor, orbOffset.Y / scaleFactor);
                        }
                        result = new OffsetResult(orbOffset, 0.8, "ORB");
                        Debug.WriteLine($"[ORB] ORB 결과: ({result.Offset.X:F3}, {result.Offset.Y:F3})");
                        break;
                    default:
                        Debug.WriteLine($"[ERROR] 색수차 방법을 다시 선택하세요: {method}");
                        result = new OffsetResult(PointF.Empty, 0, "UNKNOWN_METHOD");
                        break;
                }
                Debug.WriteLine($"[이미지 저장 경로]: {saveDir}");

                // 3. 스케일 보정
                if (optionParam.IsScaleEstimationEnabled)
                {
                    Rectangle refAlignedROI = roiRect;
                    Rectangle targetAlignedROI = new Rectangle(
                        roiRect.X + (int)result.Offset.X,
                        roiRect.Y + (int)result.Offset.Y,
                        roiRect.Width,
                        roiRect.Height
                    );

                    /*var patchRef = ExtractRegion(referenceChannel, refAlignedROI);
                    var patchTgt = ExtractRegion(targetChannel, targetAlignedROI);
                    string debugDir = Path.Combine(saveDir, "NCC_Debug");
                    if (!Directory.Exists(debugDir))
                        Directory.CreateDirectory(debugDir);
                    SaveImage(patchRef, Path.Combine(debugDir, $"{imageIndex}_debug_ref_patch.png"));
                    SaveImage(patchTgt, Path.Combine(debugDir, $"{imageIndex}_debug_tgt_patch.png"));*/

                    var scaleResult = EstimateScaleBetweenROIs(imageIndex, referenceChannel, targetChannel, refAlignedROI, targetAlignedROI, saveDir);
                    result = result.WithScale(scaleResult);
                }

                return result;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] 색수차 오프셋 계산 오류: {ex.Message}");
                var errorResult = new OffsetResult(new PointF(0, 0), 0.0, $"ERROR: {ex.Message}");
                return errorResult;
            }
        }

        #region [Scale 계산]
        private (float scaleX, float scaleY) EstimateScaleBetweenROIs(int imageIndex, Bitmap refImg, Bitmap tgtImg, Rectangle roiRef, Rectangle roiTgt, string saveDir)
        {
            // 1. 스케일 범위 설정 (0.8 ~ 1.2)
            var scaleRange = Enumerable.Range(80, 41).Select(x => x / 100.0f);
            var bestScoreX = double.MinValue;
            var bestScoreY = double.MinValue;
            var bestScaleX = 1.0f;
            var bestScaleY = 1.0f;

            // 2. X 방향 스케일 계산
            foreach (var scale in scaleRange)
            {
                // 2-1. 타겟 이미지 X 방향 리사이즈
                var scaledTgt = ResizeImage(tgtImg, scale, 1.0f);

                // 2-2. ROI 영역 추출
                var patchRef = ExtractRegion(refImg, roiRef);
                var patchTgt = ExtractRegion(scaledTgt, roiTgt);

                // 2-3. NCC 계산
                var correlationMap = CalculateOpenCVStyleNCC(patchRef, patchTgt);
                var (maxVal, maxLoc) = FindMaxLocation(correlationMap);

                // 2-4. 최고 점수 업데이트
                if (maxVal > bestScoreX)
                {
                    bestScoreX = maxVal;
                    bestScaleX = scale;
                }

                // 2-5. 디버그 이미지 저장
                /*string debugDir = Path.Combine(saveDir, "NCC_Scale_Debug");
                if (!Directory.Exists(debugDir))
                    Directory.CreateDirectory(debugDir);
                SaveImage(patchRef, Path.Combine(debugDir, $"{imageIndex}_scaleX_{scale:F2}_ref.png"));
                SaveImage(patchTgt, Path.Combine(debugDir, $"{imageIndex}_scaleX_{scale:F2}_tgt.png"));*/

                // 2-6. 메모리 정리
                scaledTgt.Dispose();
                patchRef.Dispose();
                patchTgt.Dispose();
            }

            // 3. Y 방향 스케일 계산
            foreach (var scale in scaleRange)
            {
                // 3-1. 타겟 이미지 Y 방향 리사이즈
                var scaledTgt = ResizeImage(tgtImg, 1.0f, scale);

                // 3-2. ROI 영역 추출
                var patchRef = ExtractRegion(refImg, roiRef);
                var patchTgt = ExtractRegion(scaledTgt, roiTgt);

                // 3-3. NCC 계산
                var correlationMap = CalculateOpenCVStyleNCC(patchRef, patchTgt);
                var (maxVal, maxLoc) = FindMaxLocation(correlationMap);

                // 3-4. 최고 점수 업데이트
                if (maxVal > bestScoreY)
                {
                    bestScoreY = maxVal;
                    bestScaleY = scale;
                }

                // 3-5. 디버그 이미지 저장
                /*string debugDir = Path.Combine(saveDir, "NCC_Scale_Debug");
                if (!Directory.Exists(debugDir))
                    Directory.CreateDirectory(debugDir);
                SaveImage(patchRef, Path.Combine(debugDir, $"{imageIndex}_scaleY_{scale:F2}_ref.png"));
                SaveImage(patchTgt, Path.Combine(debugDir, $"{imageIndex}_scaleY_{scale:F2}_tgt.png"));*/

                // 3-6. 메모리 정리
                scaledTgt.Dispose();
                patchRef.Dispose();
                patchTgt.Dispose();
            }

            Debug.WriteLine($"[Scale][{imageIndex}] Best ScaleX: {bestScaleX:F3} (Score: {bestScoreX:F4}), Best ScaleY: {bestScaleY:F3} (Score: {bestScoreY:F4})");
            return (bestScaleX, bestScaleY);
        }

        private Bitmap ResizeImage(Bitmap original, float scaleX, float scaleY)
        {
            using (Mat src = BitmapToMat(original))
            using (Mat resized = new Mat())
            {
                CvInvoke.Resize(src, resized, new Size(
                    (int)(src.Width * scaleX),
                    (int)(src.Height * scaleY)
                ), interpolation: Inter.Cubic);

                return MatToBitmap(resized);
            }
        }

        #endregion

        #region [기타 함수]
        private double GetMedian(List<double> values)
        {
            var sorted = values.OrderBy(x => x).ToList();
            int count = sorted.Count;

            if (count % 2 == 1)
                return sorted[count / 2];
            else
                return (sorted[(count / 2) - 1] + sorted[count / 2]) / 2.0;
        }
        private void SaveImage(Bitmap saveImage, string savePath)
        {
            saveImage.Save(savePath, ImageFormat.Png);
        }

        /// <summary>
        /// 특징점을 시각화해서 저장하는 메서드
        /// </summary>
        private void SaveKeypointsImage(Mat image, VectorOfKeyPoint keypoints, string savePath)
        {
            try
            {
                using (var outputMat = new Mat())
                {
                    // 특징점을 이미지에 그리기
                    Features2DToolbox.DrawKeypoints(image, keypoints, outputMat, new Bgr(0, 255, 0), Features2DToolbox.KeypointDrawType.Default);
                    using (var bitmap = MatToBitmap(outputMat))
                    {
                        bitmap.Save(savePath, ImageFormat.Png);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] ORB 특징점 시각화 저장 실패 {savePath}: {ex.Message}");
            }
        }
        /// <summary>
        /// 이미지에서 특정 영역 추출
        /// </summary>
        private Bitmap ExtractRegion(Bitmap source, Rectangle region)
        {
            // 인덱싱된 픽셀 형식 문제를 해결하기 위해 24비트 RGB로 강제 생성
            var result = new Bitmap(region.Width, region.Height, PixelFormat.Format24bppRgb);
            using (Graphics g = Graphics.FromImage(result))
            {
                g.DrawImage(source, new Rectangle(0, 0, region.Width, region.Height), region, GraphicsUnit.Pixel);
            }
            return result;
        }

        /// <summary>
        /// 보정된 이미지에서 B채널을 새로 생성
        /// </summary>
        private Bitmap RegenerateBChannelFromCorrected(Bitmap correctedImage, BChanelGenerationMethod bMethod)
        {
            if (bMethod == BChanelGenerationMethod.None)
                return correctedImage; // B채널 생성하지 않음

            int width = correctedImage.Width;
            int height = correctedImage.Height;
            var result = new Bitmap(width, height);

            BitmapData correctedData = correctedImage.LockBits(
                new Rectangle(0, 0, width, height),
                ImageLockMode.ReadOnly,
                PixelFormat.Format24bppRgb);

            BitmapData resultData = result.LockBits(
                new Rectangle(0, 0, width, height),
                ImageLockMode.WriteOnly,
                PixelFormat.Format24bppRgb);

            unsafe
            {
                byte* correctedPtr = (byte*)correctedData.Scan0;
                byte* resultPtr = (byte*)resultData.Scan0;

                int stride = correctedData.Stride;

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        int idx = y * stride + x * 3;

                        byte r = correctedPtr[idx + 2];  // Red
                        byte g = correctedPtr[idx + 1];  // Green

                        // B채널 생성 방식에 따라 계산
                        byte newB;
                        switch (bMethod)
                        {
                            case BChanelGenerationMethod.Average:
                                newB = (byte)((r + g) / 2);
                                break;
                            case BChanelGenerationMethod.Weighted:
                                newB = (byte)(r * 0.3 + g * 0.7);
                                break;
                            case BChanelGenerationMethod.GreenOnly:
                                newB = g;
                                break;
                            default:
                                newB = correctedPtr[idx]; // 기존 B 유지
                                break;
                        }

                        // 결과 이미지에 픽셀 설정
                        resultPtr[idx] = newB;      // Blue
                        resultPtr[idx + 1] = g;     // Green
                        resultPtr[idx + 2] = r;     // Red
                    }
                }
            }

            correctedImage.UnlockBits(correctedData);
            result.UnlockBits(resultData);

            return result;
        }

        /// <summary>
        /// Mat을 double[,] 배열로 변환
        /// </summary>
        private double[,] MatToDoubleArray(Mat mat)
        {
            int rows = mat.Rows;
            int cols = mat.Cols;
            var result = new double[rows, cols];

            var data = new float[rows * cols];
            mat.CopyTo(data);

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    result[i, j] = data[i * cols + j];
                }
            }

            return result;
        }

        /// <summary>
        /// Mat을 Bitmap으로 변환
        /// </summary>
        private Bitmap MatToBitmap(Mat mat)
        {
            try
            {
                Bitmap bitmap;

                if (mat.NumberOfChannels == 1)
                {
                    // 그레이스케일
                    bitmap = new Bitmap(mat.Width, mat.Height, PixelFormat.Format8bppIndexed);

                    // 그레이스케일 팔레트 설정
                    var palette = bitmap.Palette;
                    for (int i = 0; i < 256; i++)
                    {
                        palette.Entries[i] = Color.FromArgb(i, i, i);
                    }
                    bitmap.Palette = palette;
                }
                else
                {
                    // 컬러
                    bitmap = new Bitmap(mat.Width, mat.Height, PixelFormat.Format24bppRgb);
                }

                var bitmapData = bitmap.LockBits(
                    new Rectangle(0, 0, bitmap.Width, bitmap.Height),
                    ImageLockMode.WriteOnly,
                    bitmap.PixelFormat);

                try
                {
                    // 행 단위로 복사
                    for (int y = 0; y < mat.Height; y++)
                    {
                        // Mat의 한 행 데이터를 가져옴
                        var rowData = new byte[mat.Width * mat.NumberOfChannels];
                        Marshal.Copy(mat.DataPointer + y * mat.Step, rowData, 0, rowData.Length);

                        // Bitmap의 해당 행에 복사
                        Marshal.Copy(rowData, 0, bitmapData.Scan0 + y * bitmapData.Stride, rowData.Length);
                    }
                }
                finally
                {
                    bitmap.UnlockBits(bitmapData);
                }

                return bitmap;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Mat을 Bitmap으로 변환 실패: {ex.Message}");
                return new Bitmap(mat.Width, mat.Height);
            }
        }

        /// <summary>
        /// Bitmap을 Mat으로 변환 (Emgu.CV 4.x 호환)
        /// </summary>
        private Mat BitmapToMat(Bitmap bitmap)
        {
            try
            {
                var bitmapData = bitmap.LockBits(
                    new Rectangle(0, 0, bitmap.Width, bitmap.Height),
                    ImageLockMode.ReadOnly,
                    bitmap.PixelFormat);

                try
                {
                    var mat = new Mat();

                    if (bitmap.PixelFormat == PixelFormat.Format8bppIndexed)
                    {
                        // 8비트 그레이스케일
                        mat = new Mat(bitmap.Height, bitmap.Width, DepthType.Cv8U, 1,
                            bitmapData.Scan0, bitmapData.Stride);
                    }
                    else if (bitmap.PixelFormat == PixelFormat.Format24bppRgb)
                    {
                        // 24비트 BGR
                        mat = new Mat(bitmap.Height, bitmap.Width, DepthType.Cv8U, 3,
                            bitmapData.Scan0, bitmapData.Stride);
                    }
                    else
                    {
                        // 다른 포맷은 24비트로 변환
                        using (var tempBitmap = new Bitmap(bitmap.Width, bitmap.Height, PixelFormat.Format24bppRgb))
                        using (var g = Graphics.FromImage(tempBitmap))
                        {
                            g.DrawImage(bitmap, 0, 0);
                            bitmap.UnlockBits(bitmapData);
                            return BitmapToMat(tempBitmap);
                        }
                    }

                    return mat.Clone(); // 복사본 반환
                }
                finally
                {
                    bitmap.UnlockBits(bitmapData);
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"Bitmap을 Mat으로 변환 실패: {ex.Message}");
                // 실패 시 빈 Mat 반환
                return new Mat(bitmap.Height, bitmap.Width, DepthType.Cv8U, 1);
            }
        }

        /// <summary>
        /// 컬러 이미지를 그레이스케일로 변환
        /// </summary>
        private Bitmap ConvertToGrayscale(Bitmap colorImage, string channelType = "G")
        {
            var grayImage = new Bitmap(colorImage.Width, colorImage.Height, PixelFormat.Format8bppIndexed);

            // 그레이스케일 팔레트 설정
            var palette = grayImage.Palette;
            for (int i = 0; i < 256; i++)
            {
                palette.Entries[i] = Color.FromArgb(i, i, i);
            }
            grayImage.Palette = palette;

            var srcData = colorImage.LockBits(
                new Rectangle(0, 0, colorImage.Width, colorImage.Height),
                ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);

            var dstData = grayImage.LockBits(
                new Rectangle(0, 0, grayImage.Width, grayImage.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format8bppIndexed);

            unsafe
            {
                byte* srcPtr = (byte*)srcData.Scan0;
                byte* dstPtr = (byte*)dstData.Scan0;

                int channelOffset;
                switch (channelType.ToUpper())
                {
                    case "R": channelOffset = 2; break; // Red
                    case "G": channelOffset = 1; break; // Green  
                    case "B": channelOffset = 0; break; // Blue
                    default: channelOffset = 1; break;  // 기본값: Green
                }

                for (int y = 0; y < colorImage.Height; y++)
                {
                    for (int x = 0; x < colorImage.Width; x++)
                    {
                        int srcOffset = y * srcData.Stride + x * 3;
                        int dstOffset = y * dstData.Stride + x;

                        // 채널 타입에 따라 올바른 채널 값 사용
                        dstPtr[dstOffset] = srcPtr[srcOffset + channelOffset];
                    }
                }
            }

            colorImage.UnlockBits(srcData);
            grayImage.UnlockBits(dstData);

            return grayImage;
        }

        /// <summary>
        /// 채널 오프셋을 적용하여 보정된 이미지 생성
        /// </summary>
        private Bitmap ApplyChromaticAberrationCorrection(Bitmap originalImage, OffsetResult rResult, OffsetResult bResult, ROI roi)
        {
            var result = new Bitmap(originalImage.Width, originalImage.Height, PixelFormat.Format24bppRgb);

            var srcData = originalImage.LockBits(new Rectangle(0, 0, originalImage.Width, originalImage.Height),
                ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
            var dstData = result.LockBits(new Rectangle(0, 0, result.Width, result.Height),
                ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);

            try
            {
                unsafe
                {
                    byte* srcPtr = (byte*)srcData.Scan0;
                    byte* dstPtr = (byte*)dstData.Scan0;

                    float scaleRx = rResult.ScaleX ?? 1.0f;
                    float scaleRy = rResult.ScaleY ?? 1.0f;
                    float scaleBx = bResult.ScaleX ?? 1.0f;
                    float scaleBy = bResult.ScaleY ?? 1.0f;

                    // ROI 중심점 계산
                    float roiCenterX = roi.X + roi.Width / 2;
                    float roiCenterY = roi.Y + roi.Height / 2;

                    for (int y = 0; y < originalImage.Height; y++)
                    {
                        for (int x = 0; x < originalImage.Width; x++)
                        {
                            int dstOffset = y * dstData.Stride + x * 3;

                            // G 채널 그대로
                            dstPtr[dstOffset + 1] = srcPtr[y * srcData.Stride + x * 3 + 1];

                            // R 채널 - ROI 중심점 기준 스케일 보정
                            float rXf = (x + rResult.Offset.X - roiCenterX) * scaleRx + roiCenterX;
                            float rYf = (y + rResult.Offset.Y - roiCenterY) * scaleRy + roiCenterY;
                            int rX = (int)Math.Round(rXf);
                            int rY = (int)Math.Round(rYf);
                            if (rX >= 0 && rX < originalImage.Width && rY >= 0 && rY < originalImage.Height)
                                dstPtr[dstOffset + 2] = srcPtr[rY * srcData.Stride + rX * 3 + 2];

                            // B 채널 - ROI 중심점 기준 스케일 보정
                            float bXf = (x + bResult.Offset.X - roiCenterX) * scaleBx + roiCenterX;
                            float bYf = (y + bResult.Offset.Y - roiCenterY) * scaleBy + roiCenterY;
                            int bX = (int)Math.Round(bXf);
                            int bY = (int)Math.Round(bYf);
                            if (bX >= 0 && bX < originalImage.Width && bY >= 0 && bY < originalImage.Height)
                                dstPtr[dstOffset] = srcPtr[bY * srcData.Stride + bX * 3];
                        }
                    }
                }
            }
            finally
            {
                originalImage.UnlockBits(srcData);
                result.UnlockBits(dstData);
            }

            return result;
        }

        /// <summary>
        /// Bilinear Interpolation
        /// </summary>
        private unsafe double BilinearInterpolate(byte* imagePtr, int stride, double x, double y)
        {
            int x1 = (int)x;
            int y1 = (int)y;
            int x2 = x1 + 1;
            int y2 = y1 + 1;

            double fx = x - x1;
            double fy = y - y1;

            double p1 = imagePtr[y1 * stride + x1];
            double p2 = imagePtr[y1 * stride + x2];
            double p3 = imagePtr[y2 * stride + x1];
            double p4 = imagePtr[y2 * stride + x2];

            double i1 = p1 * (1 - fx) + p2 * fx;
            double i2 = p3 * (1 - fx) + p4 * fx;

            return i1 * (1 - fy) + i2 * fy;
        }

        #endregion

        #region [NCC]
        /// <summary>
        /// ROI 기반 NCC를 이용한 오프셋 계산
        /// </summary>
        private OffsetResult CalculateOffsetUsingNCCWithROIDetails(int imageIndex, Bitmap reference, Bitmap target, Rectangle roiRect, string saveDir)
        {
            Debug.WriteLine($"=== {imageIndex} NCC ROI 매칭 시작 - ROI: {roiRect.X},{roiRect.Y},{roiRect.Width},{roiRect.Height} ===");

            // 1. Template 설정
            var template = ExtractRegion(reference, roiRect);

            // 2. Search Area 설정 (최소 10 or 사용자가 선택한 roi의 1/4 사이즈)
            int expandSize = Math.Max(10, Math.Min(roiRect.Width, roiRect.Height) / 4);
            int searchX1 = Math.Max(0, roiRect.X - expandSize);
            int searchY1 = Math.Max(0, roiRect.Y - expandSize);
            int searchX2 = Math.Min(target.Width, roiRect.X + roiRect.Width + expandSize);
            int searchY2 = Math.Min(target.Height, roiRect.Y + roiRect.Height + expandSize);
            var searchAreaRect = new Rectangle(searchX1, searchY1, searchX2 - searchX1, searchY2 - searchY1);
            var searchArea = ExtractRegion(target, searchAreaRect);

            // +) 원본 디버깅용 이미지 저장
            try
            {
                string debugDir = Path.Combine(saveDir, "NCC_Debug");
                if (!Directory.Exists(debugDir))
                    Directory.CreateDirectory(debugDir);

                // Template 저장
                SaveImage(template, Path.Combine(debugDir, $"{imageIndex}_Template.png"));

                // SearchArea 저장
                SaveImage(searchArea, Path.Combine(debugDir, $"{imageIndex}_SearchArea.png"));
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] 디버그 이미지 저장 실패: {ex.Message}");
            }

            // 3. NCC 처리 진행
            try
            {
                // (0) 이미지 전처리
                /*var templateEnhanced = ApplyImageEnhancement(template);
                var searchAreaEnhanced = ApplyImageEnhancement(searchArea);*/

                // (1) NCC 계산
                // var correlationMap = CalculateOpenCVStyleNCC(templateEnhanced, searchAreaEnhanced);
                var correlationMap = CalculateOpenCVStyleNCC(template, searchArea);

                // (2) 최대 상관 위치 찾기
                var (maxVal, maxLoc) = FindMaxLocation(correlationMap);
                Debug.WriteLine($"[NCC] Result 상관관계: {maxVal:F4}, 위치: ({maxLoc.X}, {maxLoc.Y})");

                // (3) 서브픽셀 정밀도로 위치 보정
                var subPixelLoc = CalculateSubPixelLocation(correlationMap, maxLoc);
                Debug.WriteLine($"[NCC] 서브픽셀 조정 위치: ({subPixelLoc.X:F2}, {subPixelLoc.Y:F2})");

                // (4) 실제 오프셋 계산
                float offsetX = 0;
                float offsetY = 0;
                if (subPixelLoc.X != 0 && subPixelLoc.Y != 0)
                {
                    offsetX = subPixelLoc.X + searchX1 - roiRect.X;
                    offsetY = subPixelLoc.Y + searchY1 - roiRect.Y;
                }
                Debug.WriteLine($"[NCC] Result 시프트: dx={offsetX:F2}, dy={offsetY:F2}");

                // (5) 메모리 정리
                template.Dispose();
                searchArea.Dispose();

                return new OffsetResult(new PointF(offsetX, offsetY), maxVal, $"ROI NCC (상관관계: {maxVal:F4})");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] ROI 기반 NCC 계산 실패: {ex.Message}");
                template?.Dispose();
                searchArea?.Dispose();

                return new OffsetResult(new PointF(0, 0), 0, $"NCC fallback (오류: {ex.Message})");
            }
        }

        /// <summary>
        /// OpenCV의 MatchTemplate을 사용한 NCC 계산
        /// </summary>
        private double[,] CalculateOpenCVStyleNCC(Bitmap template, Bitmap searchArea)
        {
            try
            {
                using (var templateMat = BitmapToMat(template))
                using (var searchMat = BitmapToMat(searchArea))
                {
                    using (var templateGray = new Mat())
                    using (var searchGray = new Mat())
                    {
                        // 그레이스케일 변환
                        if (templateMat.NumberOfChannels == 3)
                        {
                            CvInvoke.CvtColor(templateMat, templateGray, ColorConversion.Bgr2Gray);
                        }
                        else
                        {
                            templateMat.CopyTo(templateGray);
                        }

                        if (searchMat.NumberOfChannels == 3)
                        {
                            CvInvoke.CvtColor(searchMat, searchGray, ColorConversion.Bgr2Gray);
                        }
                        else
                        {
                            searchMat.CopyTo(searchGray);
                        }

                        using (var result = new Mat())
                        {
                            CvInvoke.MatchTemplate(searchGray, templateGray, result, TemplateMatchingType.CcoeffNormed);
                            var correlationArray = MatToDoubleArray(result);

                            // 상관관계 검증
                            double maxCorrelation = double.MinValue;
                            for (int y = 0; y < correlationArray.GetLength(0); y++)
                            {
                                for (int x = 0; x < correlationArray.GetLength(1); x++)
                                {
                                    if (correlationArray[y, x] > maxCorrelation)
                                        maxCorrelation = correlationArray[y, x];
                                }
                            }
                            return correlationArray;
                        }
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] OpenCV NCC 실패: {ex.Message}");
                return new double[1, 1] { { -1 } };
            }
        }

        /// <summary>
        /// 상관관계 맵에서 최대값의 위치 찾기
        /// </summary>
        private (double maxVal, Point maxLoc) FindMaxLocation(double[,] correlationMap)
        {
            int height = correlationMap.GetLength(0);
            int width = correlationMap.GetLength(1);

            double maxVal = double.MinValue;
            Point maxLoc = new Point(0, 0);

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    double val = correlationMap[y, x];

                    if (val > maxVal)
                    {
                        maxVal = val;
                        maxLoc = new Point(x, y);
                    }
                }
            }
            return (maxVal, maxLoc);
        }

        /// <summary>
        /// 2차 다항식 피팅으로 서브픽셀 정밀도 계산
        /// </summary>
        private PointF CalculateSubPixelLocation(double[,] correlationMap, Point maxLoc)
        {
            int height = correlationMap.GetLength(0);
            int width = correlationMap.GetLength(1);

            float subX = maxLoc.X;
            float subY = maxLoc.Y;

            // X 방향 서브픽셀 위치 추정
            if (maxLoc.X > 0 && maxLoc.X < width - 1)
            {
                double[] xNeighbors = {
                    correlationMap[maxLoc.Y, maxLoc.X - 1],
                    correlationMap[maxLoc.Y, maxLoc.X],
                    correlationMap[maxLoc.Y, maxLoc.X + 1]
                };

                double denominator = xNeighbors[0] - 2 * xNeighbors[1] + xNeighbors[2];

                if (Math.Abs(denominator) > 1e-10)
                {
                    double offset = 0.5 * (xNeighbors[0] - xNeighbors[2]) / denominator;

                    if (Math.Abs(offset) <= 1.0) // 수정 가능
                    {
                        subX = maxLoc.X + (float)offset;
                    }
                }
            }

            // Y 방향 서브픽셀 위치 추정
            if (maxLoc.Y > 0 && maxLoc.Y < height - 1)
            {
                double[] yNeighbors = {
                    correlationMap[maxLoc.Y - 1, maxLoc.X],
                    correlationMap[maxLoc.Y, maxLoc.X],
                    correlationMap[maxLoc.Y + 1, maxLoc.X]
                };

                double denominator = yNeighbors[0] - 2 * yNeighbors[1] + yNeighbors[2];

                if (Math.Abs(denominator) > 1e-10)
                {
                    double offset = 0.5 * (yNeighbors[0] - yNeighbors[2]) / denominator;

                    if (Math.Abs(offset) <= 1.0) // 수정 가능
                    {
                        subY = maxLoc.Y + (float)offset;
                    }
                }
            }

            return new PointF(subX, subY);
        }

        #endregion

        #region [ORB]
        /// <summary>
        /// ORB 기반 ROI 오프셋 계산 (실제 ORB 특징점 매칭)
        /// </summary>
        private PointF CalculateOffsetUsingORBWithROI(int imageIndex, Bitmap reference, Bitmap target, Rectangle roiRect, string saveDir)
        {
            Debug.WriteLine($"=== {imageIndex} ORB ROI 매칭 시작 - ROI: {roiRect.X},{roiRect.Y},{roiRect.Width},{roiRect.Height} ===");

            // 1. Template 설정
            var template = ExtractRegion(reference, roiRect);

            // 2. Search Area 설정
            int expandSize = Math.Max(10, Math.Min(roiRect.Width, roiRect.Height) / 4);
            int searchX1 = Math.Max(0, roiRect.X - expandSize);
            int searchY1 = Math.Max(0, roiRect.Y - expandSize);
            int searchX2 = Math.Min(target.Width, roiRect.X + roiRect.Width + expandSize);
            int searchY2 = Math.Min(target.Height, roiRect.Y + roiRect.Height + expandSize);
            var searchAreaRect = new Rectangle(searchX1, searchY1, searchX2 - searchX1, searchY2 - searchY1);
            var searchArea = ExtractRegion(target, searchAreaRect);

            // +) 원본 디버깅용 이미지 저장
            string debugDir = Path.Combine(saveDir, "ORB_Debug");
            try
            {
                if (!Directory.Exists(debugDir))
                    Directory.CreateDirectory(debugDir);

                // Template 저장
                SaveImage(template, Path.Combine(debugDir, $"{imageIndex}_Original_Template.png"));

                // SearchArea 저장
                SaveImage(searchArea, Path.Combine(debugDir, $"{imageIndex}_Original_SearchArea.png"));
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] 디버그 이미지 저장 실패: {ex.Message}");
            }

            // 3. ORB 진행
            try
            {
                var ORBOffset = CalculateOffsetUsingORB(imageIndex, template, searchArea, debugDir);

                if (ORBOffset.X == 0 || ORBOffset.Y == 0)
                    return ORBOffset;

                var offsetX = ORBOffset.X + searchX1 - roiRect.X;
                var offsetY = ORBOffset.Y + searchY1 - roiRect.Y;
                return new PointF(offsetX, offsetY);
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] ROI 기반 ORB 계산 실패: {ex.Message}");
                template?.Dispose();
                searchArea?.Dispose();
                return new PointF(0, 0);
            }
        }

        /// <summary>
        /// ORB - 이미지 전처리 진행
        /// </summary>
        private PointF CalculateOffsetUsingORB(int imageIndex, Bitmap reference, Bitmap target, string savePath)
        {
            try
            {
                using (var refMat = BitmapToMat(reference))
                using (var tarMat = BitmapToMat(target))
                using (var refGray = new Mat())
                using (var tarGray = new Mat())
                {
                    // 1. 그레이스케일 변환
                    if (refMat.NumberOfChannels == 3)
                        CvInvoke.CvtColor(refMat, refGray, ColorConversion.Bgr2Gray);
                    else
                        refMat.CopyTo(refGray);

                    if (tarMat.NumberOfChannels == 3)
                        CvInvoke.CvtColor(tarMat, tarGray, ColorConversion.Bgr2Gray);
                    else
                        tarMat.CopyTo(tarGray);

                    // 2. 이미지 전처리
                    /*if (optionParam.IsPreprocessingEnabled)
                    {
                        using (var refEnhanced = new Mat())
                        using (var tarEnhanced = new Mat())
                        using (var refBlur = new Mat())
                        using (var tarBlur = new Mat())
                        {
                            // 히스토그램 평활화 적용
                            CvInvoke.EqualizeHist(refGray, refEnhanced);
                            CvInvoke.EqualizeHist(tarGray, tarEnhanced);

                            // 가우시안 블러 적용
                            CvInvoke.GaussianBlur(refEnhanced, refBlur, new System.Drawing.Size(3, 3), 0);
                            CvInvoke.GaussianBlur(tarEnhanced, tarBlur, new System.Drawing.Size(3, 3), 0);

                            // 실제 ORB 사용
                            return ProcessORBDetector(imageIndex, refBlur, tarBlur, savePath);
                        }
                    }*/
                    if (optionParam.IsPreprocessingEnabled)
                    {
                        using (var refEq = new Mat())
                        using (var tarEq = new Mat())
                        using (var refSharp = new Mat())
                        using (var tarSharp = new Mat())
                        {
                            // 1. 히스토그램 평활화
                            CvInvoke.EqualizeHist(refGray, refEq);
                            CvInvoke.EqualizeHist(tarGray, tarEq);

                            // 2. Unsharp Masking
                            using (var refBlur = new Mat())
                            using (var tarBlur = new Mat())
                            {
                                CvInvoke.GaussianBlur(refEq, refBlur, new Size(0, 0), 3);
                                CvInvoke.GaussianBlur(tarEq, tarBlur, new Size(0, 0), 3);

                                CvInvoke.AddWeighted(refEq, 1.5, refBlur, -0.5, 0, refSharp);
                                CvInvoke.AddWeighted(tarEq, 1.5, tarBlur, -0.5, 0, tarSharp);
                            }

                            return ProcessORBDetector(imageIndex, refSharp, tarSharp, savePath);
                        }
                    }
                    else
                    {
                        // 전처리 없이 ORB 사용
                        return ProcessORBDetector(imageIndex, refGray, tarGray, savePath);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] ORB 클래스 실패: {ex.Message}");
                return new PointF(0, 0);
            }
        }

        /// <summary>
        /// ORB - ORB Detector 클래스 처리 로직
        /// </summary>
        private PointF ProcessORBDetector(int imageIndex, Mat refGray, Mat tarGray, string savePath)
        {
            try
            {
                using (var orb = new ORBDetector())
                using (var refKeyPoints = new VectorOfKeyPoint())
                using (var tarKeyPoints = new VectorOfKeyPoint())
                using (var refDescriptors = new Mat())
                using (var tarDescriptors = new Mat())
                {
                    // 1. 특징점 검출 + 기술자 만들기
                    orb.DetectAndCompute(refGray, null, refKeyPoints, refDescriptors, false);
                    orb.DetectAndCompute(tarGray, null, tarKeyPoints, tarDescriptors, false);
                    Debug.WriteLine($"[OBR] ORB 특징점: G채널 {refKeyPoints.Size}개, 비교채널 {tarKeyPoints.Size}개");
                    Debug.WriteLine($"[OBR] ORB 디스크립터: G채널 {refDescriptors.Rows}x{refDescriptors.Cols}, 비교채널 {tarDescriptors.Rows}x{tarDescriptors.Cols}");

                    // 2. 특징점을 시각화해서 저장
                    SaveKeypointsImage(refGray, refKeyPoints, Path.Combine(savePath, $"{imageIndex}_keypoints_Template.png"));
                    SaveKeypointsImage(tarGray, tarKeyPoints, Path.Combine(savePath, $"{imageIndex}_keypoints_SearchArea.png"));
                    if (refKeyPoints.Size < 5 || tarKeyPoints.Size < 5)
                    {
                        Debug.WriteLine($"[ERROR] ORB 특징점 부족 - shift 0 적용: Ref={refKeyPoints.Size}, Target={tarKeyPoints.Size}");
                        return PointF.Empty;
                    }
                    if (refDescriptors.Rows == 0 || tarDescriptors.Rows == 0)
                    {
                        Debug.WriteLine("[ERROR] ORB 디스크립터 생성 실패 - shift 0 적용");
                        return PointF.Empty;
                    }

                    // 2. BF 매칭
                    using (var matcher = new BFMatcher(DistanceType.Hamming, crossCheck: false))
                    using (var knnMatches = new VectorOfVectorOfDMatch())
                    {
                        matcher.KnnMatch(refDescriptors, tarDescriptors, knnMatches, k: 2);

                        var goodMatches = new VectorOfDMatch();
                        for (int i = 0; i < knnMatches.Size; i++)
                        {
                            if (knnMatches[i].Size >= 2)
                            {
                                var m1 = knnMatches[i][0];
                                var m2 = knnMatches[i][1];
                                if (m1.Distance < 0.8 * m2.Distance)
                                {
                                    goodMatches.Push(new[] { m1 });
                                }
                            }
                        }
                        Debug.WriteLine($"[ORB] ORB Good 매칭: {goodMatches.Size}개");

                        if (goodMatches.Size < 2)
                        {
                            Debug.WriteLine($"[ERROR] ORB 매칭 부족 - shift 0 적용: {goodMatches.Size}개");
                            return PointF.Empty;
                        }

                        Size roiSize = refGray.Size;
                        return CalculateOffsetFromORBMatches(refKeyPoints, tarKeyPoints, goodMatches, roiSize);
                    }

                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[ERROR] ORB 처리 실패: {ex.Message}");
                throw;
            }
        }

        /// <summary>
        /// ORB 매칭 결과에서 오프셋 계산
        /// </summary>
        private PointF CalculateOffsetFromORBMatches(VectorOfKeyPoint refKeyPoints, VectorOfKeyPoint tarKeyPoints, VectorOfDMatch matches, Size roiRect)
        {
            var refPoints = refKeyPoints.ToArray();
            var tarPoints = tarKeyPoints.ToArray();
            
            // 1. 매칭 품질 기반 필터링
            var allMatches = new List<MDMatch>();
            for (int i = 0; i < matches.Size; i++)
            {
                allMatches.Add(matches[i]);
            }

            // 2. 유사도 거리 기준으로 정렬하고 거리 가까운 것부터 50%만 사용
            allMatches.Sort((a, b) => a.Distance.CompareTo(b.Distance));
            int goodMatchCount = Math.Max(5, allMatches.Count / 2);
            var goodMatches = allMatches.Take(goodMatchCount).ToList();

            // 3. 오프셋 계산 (QueryIdx : G채널, TrainIdx : 비교채널)
            double offsetThreshold = Math.Max(roiRect.Width, roiRect.Height) * 0.7;
            var dxList = new List<double>();
            var dyList = new List<double>();
            int filteredOutCount = 0;

            foreach (var match in goodMatches)
            {
                if (match.QueryIdx >= 0 && match.QueryIdx < refPoints.Length &&
                    match.TrainIdx >= 0 && match.TrainIdx < tarPoints.Length)
                {
                    var refPt = refPoints[match.QueryIdx].Point;
                    var tarPt = tarPoints[match.TrainIdx].Point;

                    double dx = tarPt.X - refPt.X;
                    double dy = tarPt.Y - refPt.Y;

                    if (Math.Abs(dx) < offsetThreshold && Math.Abs(dy) < offsetThreshold)
                    {
                        dxList.Add(dx);
                        dyList.Add(dy);
                    }
                    else
                    {
                        filteredOutCount++;
                    }
                }
            }

            // 4. match 점이 여러개면 평균으로 offset 계산
            if (dxList.Count > 0)
            {
                double medianDx = GetMedian(dxList);
                double medianDy = GetMedian(dyList);

                Debug.WriteLine($"[ORB] 유효 매칭: {dxList.Count}/{goodMatches.Count} (제외: {filteredOutCount}), Offset: ({medianDx:F2}, {medianDy:F2})");

                return new PointF((float)medianDx, (float)medianDy);
            }
            else
            {
                Debug.WriteLine("[ERROR] 유효한 ORB 매칭 결과 없음 - shift 0 적용");
                return PointF.Empty;
            }
        }
        #endregion
    }

    public struct ProcessingOptions
    {
        public bool IsScaleUpEnabled;
        public bool IsPreprocessingEnabled;
        public bool IsScaleEstimationEnabled;
        public int StartFrameIndex;    // 처리 시작 프레임 인덱스
        public int EndFrameIndex;      // 처리 종료 프레임 인덱스
    }
}
