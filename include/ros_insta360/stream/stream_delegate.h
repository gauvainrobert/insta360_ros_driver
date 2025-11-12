//
//  stream_delegate.h
//  CameraSDK
//
//  Created by capjason on 2020/3/18.
//

#ifndef stream_delegate_h
#define stream_delegate_h
#include "stream_types.h"
#include <ros_insta360/camera/ins_types.h>

namespace ins_camera {
	class CAMERASDK_API StreamDelegate {
	public:
		/**
		 * @brief ��Ƶ���ݵĻص��ӿ�
		 * @param data ��Ƶ����
		 * @param size ��Ƶ���ݴ�С
		 * @param timeStamp ��Ƶ���ݵ�ʱ���
		 */
		virtual void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) = 0;

		/**
		* @brief ��Ƶ���ݵĻص��ӿ�
		* @param data ��Ƶ����(h264����h265�ĸ�ʽ)
		* @param size ��Ƶ���ݴ�С
		* @param timeStamp ��Ƶ���ݵ�ʱ���
		* @param streamType ����Ҫ��ע
		* @param stream_index ���˫·���ģ�ͨ���������ȥ����
		*/
		virtual void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) = 0;

		/**
		* @brief �������ݵĻص��ӿ�
		* @param data �������ݵ����� ��ϸ���Բ������'GyroData'
		*/
		virtual void OnGyroData(const std::vector<GyroData>& data) = 0;

		/**
		* @brief �ع����ݵĻص��ӿ�
		* @param data �������ݵ����� ��ϸ���Բ������'ExposureData'
		*/
		virtual void OnExposureData(const ExposureData& data) = 0;
	};
}

#endif /* stream_delegate_h */
