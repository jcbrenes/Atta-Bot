import cv2, os
import numpy as np

def getLastFrame(videoPath):
    cap = cv2.VideoCapture(videoPath)

    if not cap.isOpened():
        raise Exception(f'Error al abrir el video ({videoPath})')

    cap.set(cv2.CAP_PROP_POS_FRAMES, cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1)

    ret, frame = cap.read()
    if not ret:
        print(f'Error al leer el último frame del video ({videoPath})')

    cap.release()

    return frame


def splitFrameHorizontal(frame):
    height, _, _ = frame.shape

    topHalf = frame[:height // 2, :]
    bottomHalf = frame[height // 2:, :]

    return topHalf, bottomHalf


def countColorPixels(frame):
    white = [255, 255, 255]
    lowerLimit = np.clip(np.array(white) - np.array([10, 10, 10]), 0, 255)
    upperLimit = np.array(white)
    mask = cv2.inRange(frame, lowerLimit, upperLimit)

    totalPixels = frame.shape[0] * frame.shape[1]
    colorPixels = totalPixels - cv2.countNonZero(mask)

    return totalPixels, colorPixels


def saveFrame(frame, filePath):
    cv2.imwrite(filePath, frame)


def overlayImages(mask, frame):
    white = [255, 255, 255]
    lowerLimit = np.clip(np.array(white) - np.array([10, 10, 10]), 0, 255)
    upperLimit = np.array(white)
    frameMask = cv2.inRange(frame, lowerLimit, upperLimit)

    resultMask = cv2.bitwise_not(frameMask)
    frame = cv2.bitwise_and(frame, frame, mask=resultMask)
    alpha = 0.4
    beta = 1.0
    result = cv2.addWeighted(mask, alpha, frame, beta, 0)

    return result


def calculateCoverageArea(totalPixels, colorPixels, maskPixels):
    validPixels = totalPixels - maskPixels
    coverageArea = (colorPixels / validPixels) * 100

    return coverageArea


def getVideos(folderPath):
    if not os.path.isdir(folderPath):
        raise Exception('Error: The provided path is not a valid directory')

    videos = []
    for file in os.listdir(folderPath):
        if file.lower().endswith('.avi'): 
            videos.append((os.path.join(folderPath, file), file))

    return videos


def main():
    videosPath = r'Videos'
    maskPath = r'mask.png'

    videos = getVideos(videosPath)
    mask = cv2.imread(maskPath)
    _, mask = splitFrameHorizontal(mask)
    totalPixels, maskPixels = countColorPixels(mask)

    coverageAreas = []

    for videoPath, videoName in videos:
        lastFrame = getLastFrame(videoPath)
        saveFrame(lastFrame, r'mask1.png')

        _, result = splitFrameHorizontal(lastFrame)
        _, colorPixels = countColorPixels(result)

        coverageArea = calculateCoverageArea(totalPixels, colorPixels, maskPixels)
        coverageAreas.append(coverageArea)
        print(f"Area explorada en el video ({videoName}): {coverageArea:.2f}%")

        if False:
            imageResult = overlayImages(mask, result)
            cv2.imshow("Resultado", imageResult)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    


if __name__ == '__main__':
    main()


