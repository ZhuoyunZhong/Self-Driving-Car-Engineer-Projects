# Building and Training Network
import pickle
import numpy as np
from keras.models import Sequential, load_model
from keras.layers.core import Flatten, Dense, Lambda, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, Cropping2D
from loss_history import LossHistory


def behavioral_cloning_model(X_train, y_train):
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

    model.compile(loss='mse', optimizer='adam')

    history = LossHistory()
    model.fit(X_train, y_train, validation_split=0.2, shuffle=True, epochs=30, batch_size=128,
                    verbose=2, callbacks=[history])

    # Plot tranning history
    history.loss_plot('epoch')

    # Save Keras model
    model.save('model.h5')


if __name__ == '__main__':
    # Load training data
    # Separated into 3 files

    with open('dataX0.p', 'rb') as f:
        X0 = pickle.load(f)
    with open('dataX1.p', 'rb') as f:
        X1 = pickle.load(f)
    with open('datay.p', 'rb') as f:
        y_train_01 = pickle.load(f)
    with open('data_enhance.p', 'rb') as f:
        X2, y_train_2 = pickle.load(f)

    X_train = np.concatenate((X0, X1, X2))
    y_train = np.concatenate((y_train_01, y_train_2))
    print(X_train.shape)

    # Apply training
    behavioral_cloning_model(X_train, y_train)