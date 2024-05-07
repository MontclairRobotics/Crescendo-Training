# Crescendo-Training, A Rewrite Of Our Codebase By Different People!


# Documentation For Poor Souls!


### YAGSL
* [YAGSL Docs (General Advice, Explanations)](https://anvilproject.org/guides/content/creating-links)
* [YAGSL JavaDocs (Specific Explanations Of Functions)](https://broncbotz3481.github.io/YAGSL/swervelib/SwerveDrive.html)
### Limelight
* [Limelight NetworkTables Dump (Explanation of values the limelight Returns)](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api)
* [Limelight-Helpers Docs (Explanation Of Methods Available in LimelightHelpers.Java)](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)

### Motors, Encoders, The Like
* [Spark Encoders JavaDocs (Almost All Encoders We Interact With)](https://codedocs.revrobotics.com/java/com/revrobotics/relativeencoder)
* [CANSparkMAX JavaDocs (Almost All Motors We Interact With, Excluding Falcons, etc)](https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax)
* [Spark PID Controller JavaDocs (Used In Shooter, Not In Sprocket)](https://codedocs.revrobotics.com/java/com/revrobotics/sparkpidcontroller)
* [SparkMax Code Examples (Great For Understanding Implementations](https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java)

### WPILIB Nonsense
* [WPILIB PID Controller Docs](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
* [WPILIB PID Controller JavaDocs](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/PIDController.html)
* [WPILIB Command Javadocs (.andThen(), .alongWith(), etc)](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html)
    * Also just the general WPILIB docs in case none of these are what you're looking for [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/introduction.html)

### Gyroscope
* [Pigeon JavaDocs](https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1sensors_1_1_pigeon_i_m_u.html) (Our Current Gyro)
* [NavX JavaDocs](https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html) (Our Old Gyro)
* Our Gyro is within the SwerveDrive object in Drivetrain.java, call .getGyro() to get the gyro object
