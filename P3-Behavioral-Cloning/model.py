# Building and Training Network
import csv
import matplotlib.image as mpimg
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.models import Sequential, load_model
from keras.layers.core import Flatten, Dense, Lambda, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, Cropping2D
from loss_history import LossHistory


def counter():
    while 1:
        for count in range(4):
            yield count
count = counter()

# Load data set
samples = []
# Load Track1
with open('./data/Track1/driving_log.csv') as f:
    reader = csv.reader(f)
    for line in reader:
        if float(line[3]) == 0:
            if not count.__next__():
                continue
        samples.append(line)
# Load Track2
with open('./data/Track2/driving_log.csv') as f:
    reader = csv.reader(f)
    for line in reader:
        samples.append(line)
# Load Enhance Data
with open('./data/Enhance/driving_log.csv') as f:
    reader = csv.reader(f)
    for line in reader:
        samples.append(line)
train_samples, validation_samples = train_test_split(samples, test_size=0.2)


def behavioral_cloning_model():
    # Transfer learning: AlexNet
    model = Sequential()

    # Data preprocessing
    # Search only interest region where lanes are
    model.add(Cropping2D(cropping=((70, 25), (0, 0)), input_shape=(160, 320, 3)))
    # Normalization
    model.add(Lambda(lambda x: x / 255.0 - 0.5))

    # Building network
    # Conv 7*7s4, 96  #Modified
    model.add(Convolution2D(96, kernel_size=(7, 7), strides=(4, 4), padding='valid', activation='relu'))
    model.add(MaxPooling2D(pool_size=(3, 3), strides=(2, 2)))
    # Conv 5*5, 256
    model.add(Convolution2D(256, kernel_size=(3, 3), padding='same', activation='relu'))
    model.add(MaxPooling2D(pool_size=(3, 3), strides=(2, 2)))
    # Conv 3*3, 384
    model.add(Convolution2D(384, kernel_size=(3, 3), padding='same', activation='relu'))
    # Conv 3*3, 384
    model.add(Convolution2D(384, kernel_size=(3, 3), padding='same', activation='relu'))
    # Conv 3*3, 256
    model.add(Convolution2D(256, kernel_size=(3, 3), padding='same', activation='relu'))
    model.add(MaxPooling2D(pool_size=(3, 3), strides=(2, 2)))
    # Dense 4096
    model.add(Flatten())
    model.add(Dense(4096, activation='relu'))
    model.add(Dropout(rate=0.5))
    # Dense 1000  #Modified
    model.add(Dense(1000, activation='relu'))
    model.add(Dropout(rate=0.5))
    # Dense 1  #Modified
    model.add(Dense(1))
    
    # Loss function and optimizer method
    model.compile(loss='mse', optimizer='adam')

    return model


def generator(samples, batch_size=128):
    while 1:  # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, len(samples), batch_size):
            batch_samples = samples[offset:offset + batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                center_image = mpimg.imread(batch_sample[0])
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield X_train, y_train


if __name__ == '__main__':
    # Load generator function
    batch_size = 256
    train_generator = generator(train_samples, batch_size=batch_size)
    validation_generator = generator(validation_samples, batch_size=batch_size)

    # Load model
    retrain = False
    if not retrain:
        model = behavioral_cloning_model()
    else:
        model = load_model('model.h5')
    history = LossHistory()

    # Start training
    model.fit_generator(train_generator, steps_per_epoch=len(train_samples)/batch_size,
                        validation_data=validation_generator, validation_steps=len(validation_samples)/batch_size,
                        epochs=15, verbose=2, callbacks=[history])
    print('training finished')

    # Plot training history
    history.loss_plot('epoch')

    # Save Keras model
    model.save('model.h5')