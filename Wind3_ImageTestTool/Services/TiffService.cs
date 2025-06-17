using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Wind3_ImageTestTool
{
    public class TiffService
    {
        /// <summary>
        /// TIFF 파일을 로드하여 Bitmap으로 반환 
        /// </summary>
        /// <param name="filePath">TIFF 파일 경로</param>
        /// <returns>로드된 Bitmap 이미지</returns>
        public Bitmap LoadSingleTiff(string filePath)
        {
            return LoadTiffFrame(filePath, 0);
        }

        /// <summary>
        /// TIFF 파일의 특정 프레임을 로드
        /// </summary>
        /// <param name="filePath">TIFF 파일 경로</param>
        /// <param name="frameIndex">프레임 인덱스 (0부터 시작)</param>
        /// <returns>로드된 Bitmap 이미지</returns>
        public Bitmap LoadTiffFrame(string filePath, int frameIndex)
        {
            try
            {
                if (!File.Exists(filePath))
                    throw new FileNotFoundException($"파일을 찾을 수 없습니다: {filePath}");

                using (var originalImage = new Bitmap(filePath))
                {
                    // 프레임 개수 확인
                    int frameCount = originalImage.GetFrameCount(FrameDimension.Page);
                    if (frameIndex >= frameCount)
                        throw new ArgumentOutOfRangeException($"프레임 인덱스가 범위를 벗어났습니다. 요청: {frameIndex}, 최대: {frameCount - 1}");

                    // 특정 프레임 선택
                    originalImage.SelectActiveFrame(FrameDimension.Page, frameIndex);

                    // 24비트 RGB 포맷으로 변환하여 새 Bitmap 생성
                    var bitmap = new Bitmap(originalImage.Width, originalImage.Height, PixelFormat.Format24bppRgb);

                    using (Graphics g = Graphics.FromImage(bitmap))
                    {
                        g.DrawImage(originalImage, 0, 0);
                    }

                    return bitmap;
                }
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"TIFF 프레임 로드 실패: {ex.Message}", ex);
            }
        }

        /// <summary>
        /// TIFF 파일의 모든 프레임을 로드
        /// </summary>
        /// <param name="filePath">TIFF 파일 경로</param>
        /// <returns>모든 프레임의 Bitmap 배열</returns>
        public Bitmap[] LoadAllTiffFrames(string filePath)
        {
            try
            {
                if (!File.Exists(filePath))
                    throw new FileNotFoundException($"파일을 찾을 수 없습니다: {filePath}");

                using (var originalImage = new Bitmap(filePath))
                {
                    int frameCount = originalImage.GetFrameCount(FrameDimension.Page);
                    var frames = new Bitmap[frameCount];

                    for (int i = 0; i < frameCount; i++)
                    {
                        originalImage.SelectActiveFrame(FrameDimension.Page, i);

                        frames[i] = new Bitmap(originalImage.Width, originalImage.Height, PixelFormat.Format24bppRgb);
                        using (Graphics g = Graphics.FromImage(frames[i]))
                        {
                            g.DrawImage(originalImage, 0, 0);
                        }
                    }

                    return frames;
                }
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"TIFF 모든 프레임 로드 실패: {ex.Message}", ex);
            }
        }

        /// <summary>
        /// TIFF 파일의 프레임 개수를 반환
        /// </summary>
        /// <param name="filePath">TIFF 파일 경로</param>
        /// <returns>프레임 개수</returns>
        public int GetFrameCount(string filePath)
        {
            try
            {
                if (!File.Exists(filePath))
                    return 0;

                using (var image = new Bitmap(filePath))
                {
                    return image.GetFrameCount(FrameDimension.Page);
                }
            }
            catch
            {
                return 0;
            }
        }

        /// <summary>
        /// 여러 Bitmap을 멀티프레임 TIFF로 저장
        /// </summary>
        /// <param name="bitmaps">저장할 Bitmap 배열</param>
        /// <param name="filePath">저장할 파일 경로</param>
        public void SaveMultiFrameTiff(Bitmap[] bitmaps, string filePath)
            {
                if (bitmaps == null || bitmaps.Length == 0)
                    throw new ArgumentException("저장할 이미지가 없습니다.");

                // 디렉토리가 없으면 생성
                string directory = Path.GetDirectoryName(filePath);
                if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

            // TIFF 인코더 찾기
                var encoder = ImageCodecInfo.GetImageEncoders()
                    .FirstOrDefault(e => e.FormatID == ImageFormat.Tiff.Guid);

                if (encoder == null)
                    throw new NotSupportedException("TIFF 인코더를 찾을 수 없습니다.");

            EncoderParameters encoderParams = new EncoderParameters(2);
            encoderParams.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.Compression, (long)EncoderValue.CompressionLZW);
            
            bool firstPage = true;
            Image tiffImage = null;
            
            lock (this)
            {
                try
                {
                    for (int i = 0; i < bitmaps.Length; i++)
                    {
                        if (bitmaps[i] == null) continue;

                        try
                        {
                            if (firstPage)
                            {
                                // 첫 번째 프레임: 파일 생성
                                encoderParams.Param[1] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.MultiFrame);
                                
                                // 첫 번째 이미지 복사본 생성
                                tiffImage = (Bitmap)bitmaps[i].Clone();
                                tiffImage.Save(filePath, encoder, encoderParams);
                                
                                firstPage = false;
                                System.Diagnostics.Debug.WriteLine($"첫 번째 프레임 저장 완료: {filePath}");
                            }
                            else
                            {
                                // 나머지 프레임: 추가
                                encoderParams.Param[1] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.FrameDimensionPage);
                                tiffImage.SaveAdd(bitmaps[i], encoderParams);
                                
                                System.Diagnostics.Debug.WriteLine($"프레임 {i + 1}/{bitmaps.Length} 추가 완료");
                            }
                            
                            // 주기적 메모리 정리 (가벼운 정리)
                            if (i % 10 == 0)
                            {
                                GC.Collect();
                                GC.WaitForPendingFinalizers();
                                
                                long memoryUsage = GC.GetTotalMemory(false);
                                System.Diagnostics.Debug.WriteLine($"메모리 사용량: {memoryUsage / 1024 / 1024:N0}MB (프레임 {i + 1}/{bitmaps.Length})");
                            }
                        }
                        catch (OutOfMemoryException ex)
                        {
                            throw new OutOfMemoryException($"프레임 {i + 1}/{bitmaps.Length} 저장 중 메모리 부족 발생: {ex.Message}", ex);
                        }
                    }

                    // 마지막 단계: Flush
                    if (tiffImage != null)
                    {
                        encoderParams.Param[1] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.Flush);
                        tiffImage.SaveAdd(encoderParams);
                        System.Diagnostics.Debug.WriteLine($"멀티프레임 TIFF 저장 완료: {bitmaps.Length}개 프레임");
                    }
                    else
                    {
                        // 빈 파일 생성
                        File.Create(filePath).Close();
                        System.Diagnostics.Debug.WriteLine("빈 TIFF 파일 생성");
                    }
                }
                catch (OutOfMemoryException ex)
                {
                    long currentMemory = GC.GetTotalMemory(false);
                    throw new InvalidOperationException($"메모리 부족으로 멀티프레임 TIFF 저장 실패.\n", ex);
                }
                catch (Exception ex)
                {
                    throw new InvalidOperationException($"멀티프레임 TIFF 저장 실패: {ex.Message}", ex);
                }
                finally
                {
                    // 리소스 정리
                    encoderParams?.Dispose();
                    tiffImage?.Dispose();
                    
                    // 최종 메모리 정리
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                }
            }
        }

        /// <summary>
        /// 대량 프레임을 배치 단위로 저장
        /// </summary>
        private void SaveMultiFrameTiffInBatches(Bitmap[] bitmaps, string filePath)
        {
            const int batchSize = 10; // 배치 크기
            string tempDir = Path.Combine(Path.GetTempPath(), "Wind3_BatchTiff_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            List<string> batchFiles = new List<string>();

            try
            {
                Directory.CreateDirectory(tempDir);
                System.Diagnostics.Debug.WriteLine($"배치 처리 임시 디렉토리: {tempDir}");
                
                // 배치 단위로 처리
                for (int i = 0; i < bitmaps.Length; i += batchSize)
                {
                    int endIndex = Math.Min(i + batchSize, bitmaps.Length);
                    var batch = new Bitmap[endIndex - i];
                    Array.Copy(bitmaps, i, batch, 0, batch.Length);

                    string batchFile = Path.Combine(tempDir, $"batch_{i}_{endIndex - 1}.tif");
                    
                    // 배치를 작은 단위로 저장 (기존 방식 사용)
                    SaveMultiFrameTiffSmall(batch, batchFile);
                    batchFiles.Add(batchFile);

                    System.Diagnostics.Debug.WriteLine($"배치 {i / batchSize + 1} 저장 완료: 프레임 {i}~{endIndex - 1}");
                    
                    // 배치 간 메모리 정리
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                }

                // 배치 파일들을 최종 파일로 병합
                MergeBatchTiffFiles(batchFiles, filePath);
                
                System.Diagnostics.Debug.WriteLine($"배치 병합 완료: {filePath}");
            }
            finally
            {
                // 임시 파일 정리
                try
                {
                    if (Directory.Exists(tempDir))
                    {
                        Directory.Delete(tempDir, true);
                        System.Diagnostics.Debug.WriteLine($"임시 디렉토리 정리: {tempDir}");
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"임시 파일 정리 실패: {ex.Message}");
                }
            }
        }

        /// <summary>
        /// 소량 프레임을 저장
        /// </summary>
        private void SaveMultiFrameTiffSmall(Bitmap[] bitmaps, string filePath)
        {
            Image tiffImage = null;
            EncoderParameters encoderParams = null;
            
            try
            {
                // 디렉토리가 없으면 생성
                string directory = Path.GetDirectoryName(filePath);
                if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                var encoder = ImageCodecInfo.GetImageEncoders()
                    .FirstOrDefault(e => e.FormatID == ImageFormat.Tiff.Guid);

                encoderParams = new EncoderParameters(1);
                encoderParams.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.MultiFrame);

                // 첫 번째 이미지로 파일 생성
                bitmaps[0].Save(filePath, encoder, encoderParams);
                GC.Collect();

                if (bitmaps.Length > 1)
                {
                    tiffImage = Image.FromFile(filePath);
                        encoderParams.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.FrameDimensionPage);

                        for (int i = 1; i < bitmaps.Length; i++)
                        {
                            tiffImage.SaveAdd(bitmaps[i], encoderParams);
                        GC.Collect();
                        }

                        encoderParams.Param[0] = new EncoderParameter(System.Drawing.Imaging.Encoder.SaveFlag, (long)EncoderValue.Flush);
                        tiffImage.SaveAdd(encoderParams);
                    }
                }
            finally
            {
                encoderParams?.Dispose();
                tiffImage?.Dispose();
                GC.Collect();
                GC.WaitForPendingFinalizers();
            }
        }

        /// <summary>
        /// 배치 TIFF 파일들을 병합
        /// </summary>
        private void MergeBatchTiffFiles(List<string> batchFiles, string targetFile)
            {
            var allFrames = new List<Bitmap>();
            
            try
            {
                foreach (string batchFile in batchFiles)
                {
                    var frames = LoadAllTiffFrames(batchFile);
                    allFrames.AddRange(frames);
                    
                    GC.Collect();
                }

                SaveMultiFrameTiffSmall(allFrames.ToArray(), targetFile);
            }
            finally
            {
                foreach (var frame in allFrames)
                {
                    frame?.Dispose();
                }
                GC.Collect();
                GC.WaitForPendingFinalizers();
            }
        }

        /// <summary>
        /// Bitmap을 TIFF 파일로 저장
        /// </summary>
        /// <param name="bitmap">저장할 Bitmap 이미지</param>
        /// <param name="filePath">저장할 파일 경로</param>
        public void SaveTiff(Bitmap bitmap, string filePath)
        {
            try
            {
                // 디렉토리가 없으면 생성
                string directory = Path.GetDirectoryName(filePath);
                if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
                {
                    Directory.CreateDirectory(directory);
                }

                // TIFF 형식으로 저장
                bitmap.Save(filePath, ImageFormat.Tiff);
            }
            catch (Exception ex)
            {
                throw new InvalidOperationException($"TIFF 파일 저장 실패: {ex.Message}", ex);
            }
        }

        /// <summary>
        /// TIFF 파일의 정보를 문자열로 반환
        /// </summary>
        /// <param name="filePath">TIFF 파일 경로</param>
        /// <returns>파일 정보 문자열</returns>
        public string GetTiffInfo(string filePath)
        {
            try
            {
                if (!File.Exists(filePath))
                    return "파일을 찾을 수 없습니다.";

                using (var image = new Bitmap(filePath))
                {
                    var fileInfo = new FileInfo(filePath);
                    var pixelFormat = image.PixelFormat.ToString();
                    var bitsPerPixel = GetBitsPerPixel(image.PixelFormat);
                    var frameCount = image.GetFrameCount(FrameDimension.Page);

                    return $"크기: {image.Width}×{image.Height}, " +
                           $"프레임: {frameCount}장, " +
                           $"포맷: {pixelFormat}, " +
                           $"비트깊이: {bitsPerPixel}bit, " +
                           $"파일크기: {fileInfo.Length / 1024:N0}KB";
                }
            }
            catch (Exception ex)
            {
                return $"파일 정보 읽기 실패: {ex.Message}";
            }
        }

        /// <summary>
        /// 픽셀 포맷에서 비트 수를 계산
        /// </summary>
        /// <param name="pixelFormat">픽셀 포맷</param>
        /// <returns>픽셀당 비트 수</returns>
        private int GetBitsPerPixel(PixelFormat pixelFormat)
        {
            switch (pixelFormat)
            {
                case PixelFormat.Format1bppIndexed:
                    return 1;
                case PixelFormat.Format8bppIndexed:
                    return 8;
                case PixelFormat.Format16bppGrayScale:
                case PixelFormat.Format16bppRgb555:
                case PixelFormat.Format16bppRgb565:
                case PixelFormat.Format16bppArgb1555:
                    return 16;
                case PixelFormat.Format24bppRgb:
                    return 24;
                case PixelFormat.Format32bppRgb:
                case PixelFormat.Format32bppArgb:
                case PixelFormat.Format32bppPArgb:
                    return 32;
                case PixelFormat.Format48bppRgb:
                    return 48;
                case PixelFormat.Format64bppArgb:
                case PixelFormat.Format64bppPArgb:
                    return 64;
                default:
                    return 24; // 기본값
            }
        }

        /// <summary>
        /// TIFF 파일이 유효한지 확인
        /// </summary>
        /// <param name="filePath">확인할 파일 경로</param>
        /// <returns>유효한 TIFF 파일 여부</returns>
        public bool IsValidTiff(string filePath)
        {
            try
            {
                if (!File.Exists(filePath))
                    return false;

                using (var image = new Bitmap(filePath))
                {
                    return image.Width > 0 && image.Height > 0;
                }
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// 대용량 멀티프레임 TIFF를 청크 단위로 저장
        /// </summary>
        /// <param name="bitmaps">저장할 Bitmap 배열</param>
        /// <param name="filePath">저장할 파일 경로</param>
        /// <param name="chunkSize">한 번에 처리할 프레임 수</param>
        /// <param name="progressCallback">진행률 콜백</param>
        public void SaveMultiFrameTiffSafely(Bitmap[] bitmaps, string filePath, int chunkSize = 10, Action<int, string> progressCallback = null)
        {
            if (bitmaps == null || bitmaps.Length == 0)
                throw new ArgumentException("저장할 이미지가 없습니다.");

            progressCallback?.Invoke(0, "메모리 사용량 분석 중...");

            // 메모리 사용량 체크
            long totalMemoryNeeded = EstimateMemoryUsage(bitmaps);
            long availableMemory = GC.GetTotalMemory(false);
            
            System.Diagnostics.Debug.WriteLine($"예상 메모리 사용량: {totalMemoryNeeded / 1024 / 1024:N0}MB");
            System.Diagnostics.Debug.WriteLine($"현재 사용 메모리: {availableMemory / 1024 / 1024:N0}MB");

            // 메모리가 부족할 것으로 예상되면 더 작은 청크 사용
            if (totalMemoryNeeded > availableMemory * 0.8)
            {
                chunkSize = Math.Max(1, chunkSize / 2);
                System.Diagnostics.Debug.WriteLine($"메모리 부족 예상 - 청크 크기를 {chunkSize}로 감소");
                progressCallback?.Invoke(5, $"메모리 부족 감지 - 청크 크기 조정: {chunkSize}");
            }

            try
            {
                progressCallback?.Invoke(10, "표준 방식으로 저장 시도 중...");
                // 표준 방식으로 시도
                SaveMultiFrameTiff(bitmaps, filePath);
                progressCallback?.Invoke(100, "저장 완료");
            }
            catch (OutOfMemoryException)
            {
                System.Diagnostics.Debug.WriteLine("표준 방식 실패 - 청크 방식으로 재시도");
                progressCallback?.Invoke(15, "메모리 부족 - 청크 방식으로 재시도 중...");
                
                // 메모리 정리 후 청크 방식으로 재시도
                GC.Collect();
                GC.WaitForPendingFinalizers();
                GC.Collect();
                
                SaveMultiFrameTiffInChunks(bitmaps, filePath, chunkSize, progressCallback);
            }
        }

        /// <summary>
        /// 청크 단위로 멀티프레임 TIFF 저장
        /// </summary>
        private void SaveMultiFrameTiffInChunks(Bitmap[] bitmaps, string filePath, int chunkSize, Action<int, string> progressCallback = null)
        {
            string tempDir = Path.Combine(Path.GetTempPath(), "Wind3_TiffChunks_" + Guid.NewGuid().ToString("N").Substring(0, 8));
            List<string> chunkFiles = new List<string>();

            try
            {
                Directory.CreateDirectory(tempDir);
                progressCallback?.Invoke(20, "임시 작업 디렉토리 생성 완료");
                
                int totalChunks = (int)Math.Ceiling((double)bitmaps.Length / chunkSize);
                
                // 프레임들을 청크로 나누어 임시 파일들로 저장
                for (int i = 0; i < bitmaps.Length; i += chunkSize)
                {
                    int endIndex = Math.Min(i + chunkSize, bitmaps.Length);
                    var chunk = new Bitmap[endIndex - i];
                    Array.Copy(bitmaps, i, chunk, 0, chunk.Length);

                    string chunkFile = Path.Combine(tempDir, $"chunk_{i}_{endIndex - 1}.tif");
                    SaveMultiFrameTiff(chunk, chunkFile);
                    chunkFiles.Add(chunkFile);

                    int chunkIndex = i / chunkSize + 1;
                    int chunkProgress = 20 + (int)(50.0 * chunkIndex / totalChunks);
                    progressCallback?.Invoke(chunkProgress, $"청크 {chunkIndex}/{totalChunks} 저장 완료 (프레임 {i}~{endIndex - 1})");

                    System.Diagnostics.Debug.WriteLine($"청크 저장 완료: 프레임 {i}~{endIndex - 1} -> {chunkFile}");
                    
                    // 메모리 정리
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                }

                progressCallback?.Invoke(70, "청크 파일들을 병합 중...");
                
                // 임시 파일들을 최종 파일로 병합
                MergeTiffFiles(chunkFiles, filePath, progressCallback);
                
                progressCallback?.Invoke(100, "청크 병합 및 저장 완료");
                System.Diagnostics.Debug.WriteLine($"청크 병합 완료: {filePath}");
            }
            finally
            {
                // 임시 파일 정리
                try
                {
                    if (Directory.Exists(tempDir))
                    {
                        Directory.Delete(tempDir, true);
                        System.Diagnostics.Debug.WriteLine($"임시 디렉토리 정리: {tempDir}");
                    }
                }
                catch (Exception ex)
                {
                    System.Diagnostics.Debug.WriteLine($"임시 파일 정리 실패: {ex.Message}");
                }
            }
        }

        /// <summary>
        /// 여러 TIFF 파일을 하나로 병합
        /// </summary>
        private void MergeTiffFiles(List<string> sourceFiles, string targetFile, Action<int, string> progressCallback = null)
        {
            if (sourceFiles.Count == 1)
            {
                File.Copy(sourceFiles[0], targetFile, true);
                return;
            }

            // 모든 소스 파일의 프레임들을 하나씩 로드하여 병합
            var allFrames = new List<Bitmap>();
            
            try
            {
                for (int i = 0; i < sourceFiles.Count; i++)
                {
                    var frames = LoadAllTiffFrames(sourceFiles[i]);
                    allFrames.AddRange(frames);
                    
                    int loadProgress = 70 + (int)(20.0 * (i + 1) / sourceFiles.Count);
                    progressCallback?.Invoke(loadProgress, $"청크 파일 {i + 1}/{sourceFiles.Count} 로드 완료");
                }

                progressCallback?.Invoke(90, "최종 파일 생성 중...");
                
                // 병합된 프레임들로 최종 파일 생성
                SaveMultiFrameTiff(allFrames.ToArray(), targetFile);
            }
            finally
            {
                // 로드된 프레임들 정리
                foreach (var frame in allFrames)
                {
                    frame?.Dispose();
                }
            }
        }

        /// <summary>
        /// 메모리 사용량 추정
        /// </summary>
        private long EstimateMemoryUsage(Bitmap[] bitmaps)
        {
            if (bitmaps == null || bitmaps.Length == 0) return 0;

            // 첫 번째 이미지를 기준으로 추정
            var sample = bitmaps[0];
            long bytesPerPixel = GetBitsPerPixel(sample.PixelFormat) / 8;
            long pixelsPerImage = sample.Width * sample.Height;
            long bytesPerImage = pixelsPerImage * bytesPerPixel;
            
            return bytesPerImage * bitmaps.Length * 2; // 버퍼링 고려하여 2배
        }
    }
}
