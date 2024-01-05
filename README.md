# Flashlib-Frc-Template

Project Template  for FRC robot using FlashLib

### Simulation

Run simulation with debugger:
```shell
gradlew simulateJavaDeug
```

Run `RemoteJvmDebug` task in intellij with port=5005 and host=localhost.

### Deployment

Deploy with debugger:
```shell
gradlew deploy -PdebugMode
```

Run `RemoteJvmDebug` task in intellij with port=8349 and host=roborio host.
