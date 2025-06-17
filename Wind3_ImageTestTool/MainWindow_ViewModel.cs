using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Imaging;

namespace Wind3_ImageTestTool
{
    public class MainWindow_ViewModel : ObservableObject
    {
        #region Fields
        // 서비스 클래스들
        private readonly TiffService tiffService;
        private readonly ImageProcessingService imageProcessingService;
        private readonly ChromaticAberrationService chromaticAberrationService;

        // 이미지 데이터
        private Bitmap originalImage;
        private Bitmap correctedImage;
        private string currentFilePath;

        // 멀티프레임 지원
        private Bitmap[] allFrames;
        private Bitmap[] correctedFrames;
        private int currentFrameIndex = 0;
        private int totalFrameCount = 0;

        // 성능 측정
        private Stopwatch stopwatch;
        private bool generateBChannel = true;
        public bool GenerateBChannel
        {
            get => generateBChannel;
            set
            {
                if (SetProperty(ref generateBChannel, value))
                {
                    UpdateCorrectionButtonText();
                }
            }
        }

        private BChanelGenerationMethod bChannelMethod = BChanelGenerationMethod.Average;
        public BChanelGenerationMethod BChannelMethod
        {
            get => bChannelMethod;
            set
            {
                if (SetProperty(ref bChannelMethod, value))
                {
                    UpdateCorrectionButtonText();
                }
            }
        }

        // ComboBox 바인딩을 위한 int 속성
        public int BChannelMethodIndex
        {
            get => (int)bChannelMethod - 1; // None=0은 제외하고 Average=1을 0으로 매핑
            set
            {
                if (value >= 0 && value <= 2)
                {
                    var newMethod = (BChanelGenerationMethod)(value + 1); // None은 제외하고 Average=1부터 시작
                    if (bChannelMethod != newMethod)
                    {
                        bChannelMethod = newMethod;
                        RaisePropertyChanged(nameof(BChannelMethodIndex));
                        RaisePropertyChanged(nameof(BChannelMethod));
                        UpdateCorrectionButtonText();
                    }
                }
            }
        }

        // 단일 ROI 설정
        private ROI singleROI = new ROI();

        // 프레임 네비게이션 관련 속성
        private string currentFrameText = "프레임: - / -";
        public string CurrentFrameText
        {
            get => currentFrameText;
            set => SetProperty(ref currentFrameText, value);
        }

        private string frameInputText = "1";
        public string FrameInputText
        {
            get => frameInputText;
            set => SetProperty(ref frameInputText, value);
        }

        private string frameNavigationStatus = "";
        public string FrameNavigationStatus
        {
            get => frameNavigationStatus;
            set => SetProperty(ref frameNavigationStatus, value);
        }

        private bool isPreviousFrameEnabled = false;
        public bool IsPreviousFrameEnabled
        {
            get => isPreviousFrameEnabled;
            set => SetProperty(ref isPreviousFrameEnabled, value);
        }

        private bool isNextFrameEnabled = false;
        public bool IsNextFrameEnabled
        {
            get => isNextFrameEnabled;
            set => SetProperty(ref isNextFrameEnabled, value);
        }

        private int progressValue = 0;
        public int ProgressValue
        {
            get => progressValue;
            set => SetProperty(ref progressValue, value);
        }

        private string progressText = "대기 중";
        public string ProgressText
        {
            get => progressText;
            set => SetProperty(ref progressText, value);
        }

        private string statusText = "준비 완료";
        public string StatusText
        {
            get => statusText;
            set => SetProperty(ref statusText, value);
        }

        private string processingTimeText = "처리 시간: -";
        public string ProcessingTimeText
        {
            get => processingTimeText;
            set => SetProperty(ref processingTimeText, value);
        }

        private string correctionButtonText = "2. 색수차 보정 (NCC)";
        public string CorrectionButtonText
        {
            get => correctionButtonText;
            set => SetProperty(ref correctionButtonText, value);
        }

        // Image Sources
        private BitmapSource originalImageSource;
        public BitmapSource OriginalImageSource
        {
            get => originalImageSource;
            set => SetProperty(ref originalImageSource, value);
        }

        private BitmapSource beforeCorrectionImageSource;
        public BitmapSource BeforeCorrectionImageSource
        {
            get => beforeCorrectionImageSource;
            set => SetProperty(ref beforeCorrectionImageSource, value);
        }

        private BitmapSource afterCorrectionImageSource;
        public BitmapSource AfterCorrectionImageSource
        {
            get => afterCorrectionImageSource;
            set => SetProperty(ref afterCorrectionImageSource, value);
        }

        private BitmapSource beforeRoiCorrectionImageSource;
        public BitmapSource BeforeRoiCorrectionImageSource
        {
            get => beforeRoiCorrectionImageSource;
            set => SetProperty(ref beforeRoiCorrectionImageSource, value);
        }

        private BitmapSource afterRoiCorrectionImageSource;
        public BitmapSource AfterRoiCorrectionImageSource
        {
            get => afterRoiCorrectionImageSource;
            set => SetProperty(ref afterRoiCorrectionImageSource, value);
        }

        private BitmapSource rChannelImageSource;
        public BitmapSource RChannelImageSource
        {
            get => rChannelImageSource;
            set => SetProperty(ref rChannelImageSource, value);
        }

        private BitmapSource gChannelImageSource;
        public BitmapSource GChannelImageSource
        {
            get => gChannelImageSource;
            set => SetProperty(ref gChannelImageSource, value);
        }

        private BitmapSource bChannelImageSource;
        public BitmapSource BChannelImageSource
        {
            get => bChannelImageSource;
            set => SetProperty(ref bChannelImageSource, value);
        }

        // Info Properties
        private string originalImageInfo = "이미지를 로드해주세요.";
        public string OriginalImageInfo
        {
            get => originalImageInfo;
            set => SetProperty(ref originalImageInfo, value);
        }

        private string logText = "";
        public string LogText
        {
            get => logText;
            set => SetProperty(ref logText, value);
        }

        // Button Enable States
        private bool isLoadButtonEnabled = true;
        public bool IsLoadButtonEnabled
        {
            get => isLoadButtonEnabled;
            set => SetProperty(ref isLoadButtonEnabled, value);
        }

        private bool isCorrectionButtonEnabled = false;
        public bool IsCorrectionButtonEnabled
        {
            get => isCorrectionButtonEnabled;
            set => SetProperty(ref isCorrectionButtonEnabled, value);
        }

        private bool isExportButtonEnabled = false;
        public bool IsExportButtonEnabled
        {
            get => isExportButtonEnabled;
            set => SetProperty(ref isExportButtonEnabled, value);
        }

        private bool isScaleUpEnabled = false;
        public bool IsScaleUpEnabled
        {
            get => isScaleUpEnabled;
            set
            {
                if (SetProperty(ref isScaleUpEnabled, value))
                {
                    optionParam.IsScaleUpEnabled = value;
                    chromaticAberrationService.SetOptions(optionParam);
                }
            }
        }

        private bool isPreprocessingEnabled = false;
        public bool IsPreprocessingEnabled
        {
            get => isPreprocessingEnabled;
            set
            {
                if (SetProperty(ref isPreprocessingEnabled, value))
                {
                    optionParam.IsPreprocessingEnabled = value;
                    chromaticAberrationService.SetOptions(optionParam);
                }
            }
        }

        private ProcessingOptions optionParam;
        #endregion

        #region Constructor

        public MainWindow_ViewModel()
        {
            tiffService = new TiffService();
            imageProcessingService = new ImageProcessingService();
            optionParam = new ProcessingOptions();
            chromaticAberrationService = new ChromaticAberrationService();
            imageProcessingService = new ImageProcessingService();
            stopwatch = new Stopwatch();

            // Commands 초기화
            LoadTiffCommand = new RelayCommand(async () => await LoadTiffAsync(), () => IsLoadButtonEnabled);
            CorrectAberrationCommand = new RelayCommand(async () => await CorrectAberrationAsync(), () => IsCorrectionButtonEnabled);
            ExportTiffCommand = new RelayCommand(ExportTiff, () => IsExportButtonEnabled);
            PreviousFrameCommand = new RelayCommand(PreviousFrame, () => IsPreviousFrameEnabled);
            NextFrameCommand = new RelayCommand(NextFrame, () => IsNextFrameEnabled);
            GoToFrameCommand = new RelayCommand(GoToFrame, () => IsNextFrameEnabled || IsPreviousFrameEnabled);
            ClearLogCommand = new RelayCommand(ClearLog);
        }

        #endregion

        #region Commands Implementation

        /// <summary>
        /// TIF 이미지 로드
        /// </summary>
        private async Task LoadTiffAsync()
        {
            try
            {
                OpenFileDialog openFileDialog = new OpenFileDialog
                {
                    Title = "TIF 이미지 선택",
                    Filter = "TIFF 파일|*.tif;*.tiff|모든 파일|*.*",
                    Multiselect = false
                };

                if (openFileDialog.ShowDialog() == true)
                {
                    currentFilePath = openFileDialog.FileName;
                    await LoadTiffImageAsync(currentFilePath);
                }
            }
            catch (Exception ex)
            {
                ShowError($"이미지 로드 실패: {ex.Message}");
            }
        }

        /// <summary>
        /// 색수차 보정
        /// </summary>
        private async Task CorrectAberrationAsync()
        {
            if (originalImage == null)
            {
                MessageBox.Show("먼저 이미지를 로드해주세요.", "알림", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            try
            {
                await PerformChromaticAberrationCorrectionAsync();
            }
            catch (Exception ex)
            {
                ShowError($"색수차 보정 실패: {ex.Message}");
            }
        }

        /// <summary>
        /// TIF 추출
        /// </summary>
        private void ExportTiff()
        {
            if (correctedImage == null)
            {
                MessageBox.Show("먼저 색수차 보정을 수행해주세요.", "알림", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            try
            {
                SaveFileDialog saveFileDialog = new SaveFileDialog
                {
                    Title = "보정된 이미지 저장",
                    Filter = "TIFF 파일|*.tif;*.tiff",
                    DefaultExt = "tif",
                    FileName = Path.GetFileNameWithoutExtension(currentFilePath) + "_corrected.tif"
                };

                if (saveFileDialog.ShowDialog() == true)
                {
                    // 저장 전 메모리 정리
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                    
                    if (correctedFrames != null && correctedFrames.Length > 1)
                    {
                        SetUIState(false, "멀티프레임 TIFF 저장 중...", 0);
                        
                        try
                        {
                            // 진행률 콜백 정의
                            Action<int, string> progressCallback = (progress, message) =>
                            {
                                Application.Current.Dispatcher.Invoke(() =>
                                {
                                    SetProgress(progress, message);
                                });
                            };

                            tiffService.SaveMultiFrameTiffSafely(correctedFrames, saveFileDialog.FileName, 10, progressCallback);
                            LogMessage($"TIFF가 저장되었습니다: {saveFileDialog.FileName} ({correctedFrames.Length}개 프레임)");
                            StatusText = $"멀티프레임 이미지 저장 완료 ({correctedFrames.Length}개 프레임)";
                            MessageBox.Show($"모든 보정된 프레임({correctedFrames.Length}개)이 성공적으로 저장되었습니다!\n", "완료", MessageBoxButton.OK, MessageBoxImage.Information);
                        }
                        catch (OutOfMemoryException)
                        {
                            MessageBox.Show($"메모리 부족으로 인해 멀티프레임 저장에 실패했습니다.\n", "메모리 부족", MessageBoxButton.OK, MessageBoxImage.Warning);
                            throw;
                        }
                    }
                    else
                    {
                        tiffService.SaveTiff(correctedImage, saveFileDialog.FileName);
                        LogMessage($"보정된 이미지가 저장되었습니다: {saveFileDialog.FileName}");
                        StatusText = "이미지 저장 완료";
                        MessageBox.Show("이미지가 성공적으로 저장되었습니다!", "완료", MessageBoxButton.OK, MessageBoxImage.Information);
                    }
                    
                    SetUIState(true);
                    GC.Collect();
                    GC.WaitForPendingFinalizers();
                }
            }
            catch (Exception ex)
            {
                SetUIState(true);
                
                string errorMessage = $"이미지 저장 실패: {ex.Message}";
                if (ex.InnerException != null)
                {
                    errorMessage += $"\n\n상세 정보: {ex.InnerException.Message}";
                }
                ShowError(errorMessage);
            }
        }
        #endregion

        #region Private Methods

        /// <summary>
        /// TIFF 이미지 로드 (비동기)
        /// </summary>
        private async Task LoadTiffImageAsync(string filePath)
        {
            stopwatch.Restart();
            SetUIState(false, "이미지 로딩 중...", 0);

            try
            {
                await Task.Run(() =>
                {
                    string fileInfo = tiffService.GetTiffInfo(filePath);
                    totalFrameCount = tiffService.GetFrameCount(filePath);

                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        OriginalImageInfo = $"파일: {Path.GetFileName(filePath)} | {fileInfo}";
                        SetProgress(15, "TIFF 파일 분석 완료");
                    });

                    // 모든 프레임 로드
                    if (allFrames != null)
                    {
                        foreach (var frame in allFrames)
                            frame?.Dispose();
                    }
                    
                    // 기존 보정된 프레임들 정리
                    if (correctedFrames != null)
                    {
                        foreach (var frame in correctedFrames)
                            frame?.Dispose();
                        correctedFrames = null;
                    }
                    
                    allFrames = tiffService.LoadAllTiffFrames(filePath);
                    currentFrameIndex = 0;

                    Application.Current.Dispatcher.Invoke(() => SetProgress(50, "모든 프레임 로드 완료"));

                    // 첫 번째 프레임을 현재 이미지로 설정
                    originalImage?.Dispose();
                    originalImage = new Bitmap(allFrames[0]);

                    Application.Current.Dispatcher.Invoke(() => SetProgress(75, "이미지 로드 완료"));
                });

                // UI 업데이트
                DisplayOriginalImage();
                DisplayChannelImages();
                UpdateFrameNavigation();

                stopwatch.Stop();
                SetProgress(100, "로드 완료");
                ProcessingTimeText = $"처리 시간: {stopwatch.ElapsedMilliseconds}ms";
                StatusText = "이미지 로드 완료 - 색수차 보정을 시작할 수 있습니다.";

                IsCorrectionButtonEnabled = true;
                UpdateCorrectionButtonText();

                LogMessage($"멀티프레임 TIFF 로드 완료: {Path.GetFileName(filePath)} ({totalFrameCount}프레임, {stopwatch.ElapsedMilliseconds}ms)");
            }
            finally
            {
                SetUIState(true);
            }
        }

        /// <summary>
        /// 색수차 보정 수행 (비동기) - 모든 프레임에 대해 보정 수행 (ROI 기반)
        /// </summary>
        private async Task PerformChromaticAberrationCorrectionAsync()
        {
            stopwatch.Restart();
            SetUIState(false, "색수차 보정 중...", 0);

            try
            {
                var method = IsNccSelected ? CorrectionMethod.NCC : CorrectionMethod.ORB;
                var bChannelGenerationMethod = GenerateBChannel ? BChannelMethod : BChanelGenerationMethod.None;

                await Task.Run(() =>
                {
                    var correctedFrames = new Bitmap[allFrames.Length];
                    var threadLogs = new string[allFrames.Length];
                    var logLock = new object();

                    Parallel.For(0, allFrames.Length, frameIndex =>
                    {
                        var roiResult = chromaticAberrationService.CorrectChromaticAberrationWithROIDetails(
                            allFrames[frameIndex], method, bChannelGenerationMethod, frameIndex + 1, new ROI[] { singleROI });

                        correctedFrames[frameIndex] = roiResult.CorrectedImage;

                        string offsetInfo = "";
                        if (roiResult.RegionResults?.Length > 0)
                        {
                            var firstResult = roiResult.RegionResults[0];
                            offsetInfo = $" - R:({firstResult.RChannelOffset.X:F1},{firstResult.RChannelOffset.Y:F1})";
                        }

                        lock (logLock)
                        {
                            Application.Current.Dispatcher.Invoke(() =>
                            {
                                int progressValue = 5 + (frameIndex * 70 / allFrames.Length);
                                SetProgress(progressValue, $"프레임 {frameIndex + 1}/{allFrames.Length} 보정 중...");
                                LogMessage($"프레임 {frameIndex + 1}: ROI 기반 색수차 보정 완료{offsetInfo}");
                            });
                        }
                    });

                    correctedImage?.Dispose();
                    correctedImage = new Bitmap(correctedFrames[currentFrameIndex]);

                    if (this.correctedFrames != null)
                    {
                        foreach (var frame in this.correctedFrames)
                            frame?.Dispose();
                    }
                    this.correctedFrames = correctedFrames;

                    Application.Current.Dispatcher.Invoke(() =>
                    {
                        SetProgress(75, "모든 프레임 보정 완료");
                        LogMessage($"멀티프레임 색수차 보정 완료: 총 {allFrames.Length}개 프레임 처리됨");
                    });
                });

                DisplayCorrectionResults();

                stopwatch.Stop();
                ProcessingTimeText = $"처리 시간: {stopwatch.ElapsedMilliseconds}ms";

                string bChannelInfo = GenerateBChannel ? $"B채널 생성({BChannelMethod})" : "기존 B채널 사용";
                StatusText = allFrames?.Length > 1 ?
                    $"모든 프레임({allFrames.Length}개) 색수차 보정 완료 ({bChannelInfo}) - 이미지를 저장할 수 있습니다." :
                    $"색수차 보정 완료 ({bChannelInfo}) - 이미지를 저장할 수 있습니다.";

                IsExportButtonEnabled = true;

                LogMessage($"색수차 보정 완료 - {(allFrames?.Length > 1 ? $"총 {allFrames.Length}개 프레임" : "단일 프레임")} " +
                          $"({bChannelInfo}) ({stopwatch.ElapsedMilliseconds}ms)");
            }
            finally
            {
                SetUIState(true);
            }
        }

        /// <summary>
        /// 원본 이미지 표시
        /// </summary>
        private void DisplayOriginalImage()
        {
            if (originalImage != null)
            {
                if (!singleROI.IsEmpty)
                {
                    UpdateROIVisualization();
                }
                else
                {
                    OriginalImageSource = imageProcessingService.MatToImageSource(originalImage);
                }
            }
        }

        /// <summary>
        /// RGB 채널 이미지 표시
        /// </summary>
        private void DisplayChannelImages()
        {
            if (originalImage != null)
            {
                try
                {
                    Bitmap[] channels = imageProcessingService.SplitChannels(originalImage);

                    RChannelImageSource = imageProcessingService.MatToImageSource(channels[0]);
                    GChannelImageSource = imageProcessingService.MatToImageSource(channels[1]);
                    BChannelImageSource = imageProcessingService.MatToImageSource(channels[2]);

                    foreach (var channel in channels)
                        channel?.Dispose();
                }
                catch (Exception ex)
                {
                    LogMessage($"채널 표시 중 오류: {ex.Message}");
                }
            }
        }

        /// <summary>
        /// 보정 결과 이미지 표시
        /// </summary>
        private void DisplayCorrectionResults()
        {
            BeforeCorrectionImageSource = imageProcessingService.MatToImageSource(originalImage);
            AfterCorrectionImageSource = imageProcessingService.MatToImageSource(correctedImage);

            // ROI 보정결과 이미지 표시
            if (!singleROI.IsEmpty && originalImage != null && correctedImage != null)
            {
                try
                {
                    // ROI 영역 추출
                    Bitmap beforeRoi = imageProcessingService.ExtractROI(originalImage, singleROI);
                    Bitmap afterRoi = imageProcessingService.ExtractROI(correctedImage, singleROI);

                    // ROI 보정결과 이미지 표시
                    BeforeRoiCorrectionImageSource = imageProcessingService.MatToImageSource(beforeRoi);
                    AfterRoiCorrectionImageSource = imageProcessingService.MatToImageSource(afterRoi);

                    // 메모리 해제
                    beforeRoi?.Dispose();
                    afterRoi?.Dispose();
                }
                catch (Exception ex)
                {
                    LogMessage($"ROI 보정결과 표시 중 오류: {ex.Message}");
                }
            }
            else
            {
                BeforeRoiCorrectionImageSource = null;
                AfterRoiCorrectionImageSource = null;
            }
        }

        /// <summary>
        /// 보정 버튼 텍스트 업데이트
        /// </summary>
        private void UpdateCorrectionButtonText()
        {
            string method = IsNccSelected ? "NCC" : "ORB";
            string bChannelMode = GenerateBChannel ? GetBChannelMethodText() : "기존 B채널";
            CorrectionButtonText = $"2. 색수차 보정 ({method}, {bChannelMode})";
        }

        /// <summary>
        /// B채널 생성 방법 텍스트 반환
        /// </summary>
        private string GetBChannelMethodText()
        {
            switch (BChannelMethod)
            {
                case BChanelGenerationMethod.Average:
                    return "B채널 생성(평균)";
                case BChanelGenerationMethod.Weighted:
                    return "B채널 생성(가중치)";
                case BChanelGenerationMethod.GreenOnly:
                    return "B채널 생성(G만)";
                default:
                    return "기존 B채널";
            }
        }

        /// <summary>
        /// UI 상태 설정
        /// </summary>
        private void SetUIState(bool enabled, string status = null, int progress = 0)
        {
            IsLoadButtonEnabled = enabled;

            if (originalImage != null)
                IsCorrectionButtonEnabled = enabled;

            if (correctedImage != null)
                IsExportButtonEnabled = enabled;

            if (!string.IsNullOrEmpty(status))
            {
                StatusText = status;
                ProgressText = status;
            }

            ProgressValue = progress;
        }

        /// <summary>
        /// 진행률 설정
        /// </summary>
        private void SetProgress(int value, string text)
        {
            ProgressValue = value;
            ProgressText = text;
        }

        /// <summary>
        /// 로그 메시지 추가
        /// </summary>
        private void LogMessage(string message)
        {
            string timestamp = DateTime.Now.ToString("HH:mm:ss");
            LogText += $"[{timestamp}] {message}\n";
        }

        /// <summary>
        /// 로그 초기화
        /// </summary>
        private void ClearLog()
        {
            LogText = "";
            LogMessage("로그가 초기화되었습니다.");
        }

        /// <summary>
        /// 오류 메시지 표시
        /// </summary>
        private void ShowError(string message)
        {
            LogMessage($"오류: {message}");
            MessageBox.Show(message, "오류", MessageBoxButton.OK, MessageBoxImage.Error);
            StatusText = "오류 발생";
        }

        /// <summary>
        /// 이전 프레임으로 이동
        /// </summary>
        private void PreviousFrame()
        {
            if (currentFrameIndex > 0)
            {
                currentFrameIndex--;
                LoadCurrentFrame();
                LogMessage($"이전 프레임으로 이동: {currentFrameIndex + 1}/{totalFrameCount}");
            }
        }

        /// <summary>
        /// 다음 프레임으로 이동
        /// </summary>
        private void NextFrame()
        {
            if (currentFrameIndex < totalFrameCount - 1)
            {
                currentFrameIndex++;
                LoadCurrentFrame();
                LogMessage($"다음 프레임으로 이동: {currentFrameIndex + 1}/{totalFrameCount}");
            }
        }

        /// <summary>
        /// 특정 프레임으로 이동
        /// </summary>
        private void GoToFrame()
        {
            if (int.TryParse(FrameInputText, out int frameNumber))
            {
                frameNumber--; // 1-based를 0-based로 변환
                if (frameNumber >= 0 && frameNumber < totalFrameCount)
                {
                    currentFrameIndex = frameNumber;
                    LoadCurrentFrame();
                    FrameNavigationStatus = "프레임 이동 완료";
                    LogMessage($"프레임으로 이동: {currentFrameIndex + 1}/{totalFrameCount}");
                }
                else
                {
                    FrameNavigationStatus = "잘못된 프레임 번호";
                    LogMessage($"잘못된 프레임 번호: {frameNumber + 1} (최대: {totalFrameCount})");
                }
            }
            else
            {
                FrameNavigationStatus = "숫자를 입력하세요";
            }
        }

        /// <summary>
        /// 현재 프레임 로드 및 UI 업데이트
        /// </summary>
        private void LoadCurrentFrame()
        {
            if (allFrames != null && currentFrameIndex >= 0 && currentFrameIndex < allFrames.Length)
            {
                try
                {
                    // 기존 이미지들 해제
                    originalImage?.Dispose();

                    // 현재 프레임을 원본 이미지로 설정
                    originalImage = new Bitmap(allFrames[currentFrameIndex]);

                    // UI 업데이트
                    DisplayOriginalImage();
                    DisplayChannelImages();
                    UpdateFrameNavigation();

                    // 보정된 프레임이 있는 경우 해당 프레임도 표시
                    if (correctedFrames != null && currentFrameIndex < correctedFrames.Length && correctedFrames[currentFrameIndex] != null)
                    {
                        // 보정된 이미지를 현재 프레임의 보정된 이미지로 설정
                        correctedImage?.Dispose();
                        correctedImage = new Bitmap(correctedFrames[currentFrameIndex]);
                        
                        // 보정 결과 표시
                        DisplayCorrectionResults();
                        
                        IsExportButtonEnabled = true;
                        
                        LogMessage($"프레임 {currentFrameIndex + 1}의 보정된 이미지 표시됨");
                    }
                    else
                    {
                        // 보정된 프레임이 없는 경우 보정 결과 초기화
                        correctedImage?.Dispose();
                        correctedImage = null;
                        BeforeCorrectionImageSource = null;
                        AfterCorrectionImageSource = null;
                        IsExportButtonEnabled = false;
                    }

                    FrameNavigationStatus = "";
                }
                catch (Exception ex)
                {
                    LogMessage($"프레임 로드 오류: {ex.Message}");
                    FrameNavigationStatus = "프레임 로드 실패";
                }
            }
        }

        /// <summary>
        /// 프레임 네비게이션 UI 업데이트
        /// </summary>
        private void UpdateFrameNavigation()
        {
            if (totalFrameCount > 0)
            {
                CurrentFrameText = $"프레임: {currentFrameIndex + 1} / {totalFrameCount}";
                IsPreviousFrameEnabled = currentFrameIndex > 0;
                IsNextFrameEnabled = currentFrameIndex < totalFrameCount - 1;
                FrameInputText = (currentFrameIndex + 1).ToString();
            }
            else
            {
                CurrentFrameText = "프레임: - / -";
                IsPreviousFrameEnabled = false;
                IsNextFrameEnabled = false;
                FrameInputText = "1";
            }
        }

        /// <summary>
        /// ROI 정보 업데이트
        /// </summary>
        private void UpdateROIInfo()
        {
            LogMessage("UpdateROIInfo 호출 - singleROI 상태:");
            LogMessage($"singleROI: ({singleROI.X}, {singleROI.Y}, {singleROI.Width}, {singleROI.Height}) - IsEmpty: {singleROI.IsEmpty}");
            
            if (!singleROI.IsEmpty)
            {
                LogMessage("ROI 상태: 사용자 정의 ROI 설정됨");
                
                // ROI 상태 업데이트
                if (originalImage != null)
                {
                    double xPercent = (double)singleROI.X / originalImage.Width * 100;
                    double yPercent = (double)singleROI.Y / originalImage.Height * 100;
                    double widthPercent = (double)singleROI.Width / originalImage.Width * 100;
                    double heightPercent = (double)singleROI.Height / originalImage.Height * 100;
                }
            }
            else
            {
                LogMessage("ROI 상태: 기본 ROI 사용");
            }

            // ROI 시각화 업데이트
            UpdateROIVisualization();
        }

        /// <summary>
        /// ROI 영역을 시각화하여 표시
        /// </summary>
        private void UpdateROIVisualization()
        {
            if (originalImage == null) return;
            try
            {
                var roiImage = CreateROIVisualizedImage(originalImage);
                OriginalImageSource = imageProcessingService.MatToImageSource(roiImage);
                roiImage.Dispose();
            }
            catch (Exception ex)
            {
                LogMessage($"ROI 시각화 오류: {ex.Message}");
            }
        }

        /// <summary>
        /// ROI가 표시된 이미지 생성
        /// </summary>
        private Bitmap CreateROIVisualizedImage(Bitmap originalImage)
        {
            Bitmap rgbImage = new Bitmap(originalImage.Width, originalImage.Height, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            using (var tempGraphics = Graphics.FromImage(rgbImage))
            {
                tempGraphics.DrawImage(originalImage, 0, 0);
            }
            
            using (var graphics = Graphics.FromImage(rgbImage))
            {
                // ROI 설정 확인
                ROI roi;
                if (!singleROI.IsEmpty)
                {
                    roi = singleROI;
                }
                else
                {
                    int roiWidth = (int)(originalImage.Width * 0.8);
                    int roiHeight = (int)(originalImage.Height * 0.8);
                    int roiX = (originalImage.Width - roiWidth) / 2;
                    int roiY = (originalImage.Height - roiHeight) / 2;
                    roi = new ROI(roiX, roiY, roiWidth, roiHeight);
                }

                using (var pen = new System.Drawing.Pen(System.Drawing.Color.Red, 3))
                {
                    graphics.DrawRectangle(pen, roi.X, roi.Y, roi.Width, roi.Height);
                }

                // ROI 정보 텍스트 표시
                string roiText = $"ROI: ({roi.X},{roi.Y},{roi.Width},{roi.Height})";
                using (var brush = new System.Drawing.SolidBrush(System.Drawing.Color.Red))
                using (var font = new System.Drawing.Font("Arial", 12, System.Drawing.FontStyle.Bold))
                {
                    var textSize = graphics.MeasureString(roiText, font);
                    using (var bgBrush = new System.Drawing.SolidBrush(System.Drawing.Color.FromArgb(180, System.Drawing.Color.White)))
                    {
                        graphics.FillRectangle(bgBrush, 5, 5, textSize.Width + 10, textSize.Height + 5);
                    }
                    graphics.DrawString(roiText, font, brush, 10, 10);
                }
                using (var overlayBrush = new System.Drawing.SolidBrush(System.Drawing.Color.FromArgb(50, System.Drawing.Color.Red)))
                {
                    graphics.FillRectangle(overlayBrush, roi.X, roi.Y, roi.Width, roi.Height);
                }
            }
            
            return rgbImage;
        }

        /// <summary>
        /// 색수차 보정 상세 정보 로그 출력
        /// </summary>
        private void LogChromaticAberrationDetailsFiltered(ChromaticAberrationResult[] regionResults, string frameInfo)
        {
            if (regionResults == null || regionResults.Length == 0) return;

            Application.Current.Dispatcher.Invoke(() => 
            {
                LogMessage($"========== {frameInfo} ==========");
                LogMessage($"보정 방법: {regionResults[0].Method} | B채널: {GetBChannelMethodDescription(regionResults[0].BChannelMethod)}");
                LogMessage("");

                bool hasMovement = false;
                
                for (int i = 0; i < regionResults.Length; i++)
                {
                    var result = regionResults[i];
                    
                    // R채널 이동량 계산
                    double rDistance = Math.Sqrt(result.RChannelOffset.X * result.RChannelOffset.X + result.RChannelOffset.Y * result.RChannelOffset.Y);
                    
                    // B채널 이동량 계산
                    double bDistance = 0;
                    if (result.BChannelMethod == BChanelGenerationMethod.None)
                    {
                        bDistance = Math.Sqrt(result.BChannelOffset.X * result.BChannelOffset.X + result.BChannelOffset.Y * result.BChannelOffset.Y);
                    }
                    
                    if (rDistance > 0.05 || bDistance > 0.05)
                    {
                        hasMovement = true;
                        LogMessage($"▶ {result.RegionInfo}");
                        
                        if (rDistance > 0.05)
                        {
                            LogMessage($"  R채널 이동: ({result.RChannelOffset.X:F3}, {result.RChannelOffset.Y:F3}) px (거리: {rDistance:F3} px)");
                            if (result.RChannelNCC > 0)
                                LogMessage($"           NCC: {result.RChannelNCC:F4}");
                        }
                        
                        if (bDistance > 0.05 && result.BChannelMethod == BChanelGenerationMethod.None)
                        {
                            LogMessage($"  B채널 이동: ({result.BChannelOffset.X:F3}, {result.BChannelOffset.Y:F3}) px (거리: {bDistance:F3} px)");
                            if (result.BChannelNCC > 0)
                                LogMessage($"           NCC: {result.BChannelNCC:F4}");
                        }
                        else if (result.BChannelMethod != BChanelGenerationMethod.None)
                        {
                            LogMessage($"  B채널: {GetBChannelMethodDescription(result.BChannelMethod)}로 재생성됨");
                        }
                        
                        LogMessage($"  처리 시간: {result.ProcessingTime.TotalMilliseconds:F1}ms");
                        LogMessage("");
                    }
                }
                
                if (!hasMovement)
                {
                    LogMessage("[ERROR] 모든 영역에서 색수차 이동량이 미미함 (0.05px 미만)");
                }
                else
                {
                    double totalRDistance = regionResults.Sum(r => Math.Sqrt(r.RChannelOffset.X * r.RChannelOffset.X + r.RChannelOffset.Y * r.RChannelOffset.Y));
                    double totalBDistance = regionResults.Where(r => r.BChannelMethod == BChanelGenerationMethod.None)
                        .Sum(r => Math.Sqrt(r.BChannelOffset.X * r.BChannelOffset.X + r.BChannelOffset.Y * r.BChannelOffset.Y));
                    
                    LogMessage($"요약: R채널 총 이동 {totalRDistance:F3}px" + 
                             (totalBDistance > 0 ? $", B채널 총 이동 {totalBDistance:F3}px" : ""));
                }

                LogMessage($"================================");
            });
        }

        /// <summary>
        /// B채널 생성 방법 설명 반환
        /// </summary>
        private string GetBChannelMethodDescription(BChanelGenerationMethod method)
        {
            switch (method)
            {
                case BChanelGenerationMethod.None:
                    return "기존 B채널 사용";
                case BChanelGenerationMethod.Average:
                    return "(R+G)/2 평균";
                case BChanelGenerationMethod.Weighted:
                    return "G채널 70% 가중치";
                case BChanelGenerationMethod.GreenOnly:
                    return "G채널 복사";
                default:
                    return "알 수 없음";
            }
        }

        /// <summary>
        /// 마우스 드래그로 ROI 설정
        /// </summary>
        public void UpdateROIFromPoints(System.Drawing.Point startPoint, System.Drawing.Point endPoint)
        {
            if (originalImage == null) return;

            // 이미지 좌표를 실제 픽셀 좌표로 변환
            double scaleX = (double)originalImage.Width / OriginalImageSource.Width;
            double scaleY = (double)originalImage.Height / OriginalImageSource.Height;

            int x = (int)(Math.Min(startPoint.X, endPoint.X) * scaleX);
            int y = (int)(Math.Min(startPoint.Y, endPoint.Y) * scaleY);
            int width = (int)(Math.Abs(endPoint.X - startPoint.X) * scaleX);
            int height = (int)(Math.Abs(endPoint.Y - startPoint.Y) * scaleY);

            // ROI 업데이트
            singleROI = new ROI(x, y, width, height);
            UpdateROIInfo();
        }

        #endregion

        #region Properties

        // Commands
        public ICommand LoadTiffCommand { get; }
        public ICommand CorrectAberrationCommand { get; }
        public ICommand ExportTiffCommand { get; }
        public ICommand PreviousFrameCommand { get; }
        public ICommand NextFrameCommand { get; }
        public ICommand GoToFrameCommand { get; }
        public ICommand SetSideROICommand { get; }
        public ICommand SetBottomROICommand { get; }
        public ICommand ClearLogCommand { get; }
        public ICommand SaveROIDebugCommand { get; }

        // UI Properties
        private bool isNccSelected = true;
        public bool IsNccSelected
        {
            get => isNccSelected;
            set
            {
                if (SetProperty(ref isNccSelected, value))
                {
                    UpdateCorrectionButtonText();
                }
            }
        }

        private bool isOrbSelected = false;
        public bool IsOrbSelected
        {
            get => isOrbSelected;
            set
            {
                if (SetProperty(ref isOrbSelected, value))
                {
                    UpdateCorrectionButtonText();
                }
            }
        }

        #endregion
    }
}
