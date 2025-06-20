﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using static System.Windows.Clipboard;

namespace Wind3_ImageTestTool
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class MainWindow : Window
    {
        private MainWindow_ViewModel viewModel;
        private Point? startPoint;
        private bool isCtrlPressed = false;

        public MainWindow()
        {
            InitializeComponent();
            viewModel = new MainWindow_ViewModel();
            DataContext = viewModel;
        }

        private void Image_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (Keyboard.Modifiers == ModifierKeys.Control)
            {
                isCtrlPressed = true;
                startPoint = e.GetPosition((IInputElement)sender);
            }
        }

        private void Image_MouseRightButtonDown(object sender, MouseButtonEventArgs e)
        {
            if (Keyboard.Modifiers == ModifierKeys.Control)
            {
                isCtrlPressed = true;
                startPoint = e.GetPosition((IInputElement)sender);
            }
        }

        private void Image_MouseMove(object sender, MouseEventArgs e)
        {
            if (isCtrlPressed && startPoint.HasValue)
            {
                var currentPoint = e.GetPosition((IInputElement)sender);
                var startDrawingPoint = new System.Drawing.Point((int)startPoint.Value.X, (int)startPoint.Value.Y);
                var currentDrawingPoint = new System.Drawing.Point((int)currentPoint.X, (int)currentPoint.Y);
                ((MainWindow_ViewModel)DataContext).UpdateROIFromPoints(startDrawingPoint, currentDrawingPoint);
            }
        }

        private void Image_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (isCtrlPressed)
            {
                isCtrlPressed = false;
                startPoint = null;
            }
        }

        private void Image_MouseRightButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (isCtrlPressed)
            {
                isCtrlPressed = false;
                startPoint = null;
            }
        }

        private void TabControl_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
        }

        private void NumberValidationTextBox(object sender, TextCompositionEventArgs e)
        {
            // 숫자만 입력 가능하도록 검증
            if (!int.TryParse(e.Text, out _))
            {
                e.Handled = true;
            }
        }

        private void CopyROICorrectedImage_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (ROICorrectedImage.Source is BitmapSource bitmapSource)
                {
                    // 클립보드에 직접 BitmapSource 복사
                    Clipboard.SetImage(bitmapSource);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"이미지 복사 중 오류가 발생했습니다: {ex.Message}", "오류", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void CopyROICorrectedImage2_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                if (ROIBeforeImage.Source is BitmapSource bitmapSource)
                {
                    // 클립보드에 직접 BitmapSource 복사
                    Clipboard.SetImage(bitmapSource);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"이미지 복사 중 오류가 발생했습니다: {ex.Message}", "오류", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }
    }
}
