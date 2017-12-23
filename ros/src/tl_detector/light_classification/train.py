from keras.preprocessing.image import ImageDataGenerator
from keras.layers import Activation, Dropout, Convolution2D, MaxPooling2D, Flatten, Dense
from keras.models import Sequential

BATCH_SIZE = 64
EPOCH = 10
CLASSES = 3
IMG_SHAPE = [64, 64, 3]

def train():
    model = Sequential()
    model.add(Convolution2D(32, 3, 3, border_mode='same', input_shape=IMG_SHAPE))
    model.add(Activation('relu'))
    model.add(Convolution2D(32, 3, 3))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(.25))
    model.add(Convolution2D(64, 3, 3, border_mode='same'))
    model.add(Activation('relu'))
    model.add(Convolution2D(64, 3, 3))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Dropout(.25))
    model.add(Flatten())
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dropout(.5))
    model.add(Dense(CLASSES))
    model.add(Activation('softmax'))
    model.compile(
        loss='categorical_crossentropy',
        optimizer='rmsprop',
        metrics=['acuracy'])
    data_gen = ImageDataGenerator(
        width_shift_range=.2, 
        height_shift_range=.2, 
        shear_range=.05, 
        zoom_range=.1, 
        fill_mode='nearest', 
        rescale=1. / 255)
    img_data_gen = data_gen.flow_from_directory('images', 
        target_size=(64, 64), 
        classes=['green', 'red', 'unknown'], 
        batch_size=BATCH_SIZE)
    model.fit_generator(img_data_gen, 
        nb_epoch=EPOCH, 
        samples_per_epoch=img_data_gen.nb_sample)
    model.save('ltc_model.h5')

if __name__ == '__main__':
    train()