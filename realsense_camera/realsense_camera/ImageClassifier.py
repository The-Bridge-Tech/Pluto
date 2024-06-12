# Matthew Lauriault
# AI Model that classifies a grass image as cut/uncut
# 6/12/24


# IMPORTS
import os
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator



def classify(image) -> bool:
    """Return True if grass is cut, False if grass is uncut"""
    pass



def train(dataset_name: str):
    """Train new image classification model on specified dataset.
    Return the trained model."""

    # Paths
    dataset_dir = os.path.join("datasets", dataset_name)
    train_dir = os.path.join(dataset_dir, "train")
    validation_dir = os.path.join(dataset_dir, "validation")
    test_dir = os.path.join(dataset_dir, "test")

    # Data augmentation and normalization for training
    train_datagen = ImageDataGenerator(
        rescale=1./255,  # Normalize pixel values
        rotation_range=40,
        width_shift_range=0.2,
        height_shift_range=0.2,
        shear_range=0.2,
        zoom_range=0.2,
        horizontal_flip=True,
        fill_mode='nearest'
    )

    # Rescale the validation and testing data
    validation_datagen = ImageDataGenerator(rescale=1./255)
    test_datagen = ImageDataGenerator(rescale=1./255)

    # Create data generators
    train_generator = train_datagen.flow_from_directory(
        train_dir,
        target_size=(224, 224),     # Resize images to 224x224 pixels
        batch_size=32,              # Number of samples used at each iteration
        class_mode='binary'         # Binary classification
    )
    validation_generator = validation_datagen.flow_from_directory(
        validation_dir,
        target_size=(224, 224),
        batch_size=32,
        class_mode='binary'
    )
    test_generator = test_datagen.flow_from_directory(
        test_dir,
        target_size=(224, 224),
        batch_size=32,
        class_mode='binary'
    )

    # Initialize the model's weights, shape, and classes
    model = tf.keras.applications.ResNet50(weights=None, input_shape=(224, 224, 3), classes=1)
    # Initialize the model's optimizer
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

    # Train the model using the data generators
    model.fit(
        train_generator,
        steps_per_epoch=train_generator.samples // train_generator.batch_size,
        validation_data=validation_generator,
        validation_steps=validation_generator.samples // validation_generator.batch_size,
        epochs=10
    )

    # Evaluate the model on the test dataset
    test_loss, test_acc = model.evaluate(test_generator)
    print(f'Test accuracy: {test_acc}')
    # Return the model
    return model



# When this file is run as a script
if __name__ == '__main__':
    train("set1")