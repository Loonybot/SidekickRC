# Sidekick RC
Alpha release of Sidekick RC.

### Latest Versions

| Component                | Version                                                                                              |
|--------------------------|------------------------------------------------------------------------------------------------------|
| **Sidekick Application** | ![Latest App Release](https://img.shields.io/github/v/release/Loonybot/SidekickRC?filter=app-v*)     |
| **Sidekick Library**     | ![Latest Library Release](https://img.shields.io/github/v/release/Loonybot/SidekickRC?filter=lib-v*) |

### FTC Gradle Setup

Add JitPack to your repositories:

```gradle
repositories {
    maven { url 'https://jitpack.io' }
}
```
Then use the version number from the latest Sidekick Library release to add the following to 
your build.gradle:

```
dependencies {
    implementation 'com.github.Loonybot:SidekickRC:lib-vX.Y.Z'
}
```

