# Matthew Lauriault
# AI Model that classifies a grass image as cut/uncut
# 6/12/24


# IMPORTS
import os
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Model


# CONSTANTS
if __name__ == '__main__':
    DATASET_DIR = "src/Pluto/realsense_camera/realsense_camera/datasets"
    MODEL_DIR = "src/Pluto/realsense_camera/realsense_camera/models"
else:
    DATASET_DIR = "datasets"
    MODEL_DIR = "models"


# NOTE for each dataset, fill the "test" set with actual images of grass from the camera
# NOTE the camera should be pointed slightly down on the mower for the best classification accuracy


class ImageClassifier:

    """Wraps tensorflow.keras.models.Model"""

    def __init__(self, model: Model):
        self.model = model

    def classify(self, image) -> bool:
        """Return if grass is cut."""
        prediction = self.model.predict(image)
        # Use a threshold of 0.5 to determine the class
        return prediction[0] > 0.5
    
    def save(self, filename: str):
        """Save the model to the specified filename."""
        model_dir = os.path.join(self.MODEL_DIR, filename)
        self.model.save(model_dir)


# FACTORY METHODS
        
def load(filename: str) -> ImageClassifier:
    """Load ImageClassifer model from the specified filename."""
    model_dir = os.path.join(MODEL_DIR, filename)
    return ImageClassifier(tf.keras.models.load_model(filename))

def train(dataset_filename: str, epochs: int = 10) -> ImageClassifier:
    """
    Train and evaluate new image classification model on the specified dataset
    for the specified number of epochs.
    
    Return a new ImageClassification object with the trained model.

    NOTES:
    * adjust the number of epochs based on dataset size and performance
    * all images must be large enough to be resized to 224x224 pixels
    """

    # Paths
    dataset_dir = os.path.join(DATASET_DIR, dataset_filename)
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
        class_mode='binary'         # Binary classification (1=cut, 0=uncut)
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

    # Build the model using a pre-trained ResNet50 architecture
    base_model = tf.keras.applications.ResNet50(
        weights='imagenet', 
        include_top=False, 
        input_shape=(224, 224, 3)
    )
    # Build the model's predictions (outputs)
    x = base_model.output
    x = tf.keras.layers.GlobalAveragePooling2D()(x)
    x = tf.keras.layers.Dense(1024, activation='relu')(x)
    predictions = tf.keras.layers.Dense(1, activation='sigmoid')(x)

    # initialize the model
    model = tf.keras.models.Model(inputs=base_model.input, outputs=predictions)

    # Freeze the base model's layers to only train the added layers
    for layer in base_model.layers:
        layer.trainable = False

    # Compile the model using optimizer
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

    # Train the model using the data generators
    model.fit(
        train_generator,
        steps_per_epoch=train_generator.samples // train_generator.batch_size,
        validation_data=validation_generator,
        validation_steps=validation_generator.samples // validation_generator.batch_size,
        epochs=epochs
    )

    # Evaluate the model on the test dataset
    test_loss, test_acc = model.evaluate(test_generator)
    print(f'Test accuracy: {test_acc}')
    # Return new ImageClassifer model
    return ImageClassifier(model)



# TESTS

def testDirs():
    print(os.listdir(os.getcwd()))
    print(os.listdir(DATASET_DIR))
    print(os.listdir(MODEL_DIR))

def testTrain():
    model = train("set1")
    model.save("model1")



# If this file is run as a script
if __name__ == '__main__':
    testDirs()
    testTrain()