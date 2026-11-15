// swift-tools-version: 6.0

import PackageDescription

let package = Package(
    name: "StandardCyborgFusion",
    platforms: [
        .iOS(.v16), .macOS(.v12)
    ],
    products: [
        .library(
            name: "StandardCyborgFusion",
            type: .dynamic,
            targets: ["StandardCyborgFusion"]
        ),
    ],
    dependencies: [
        .package(url: "https://github.com/ZipArchive/ZipArchive.git", from: "2.6.0"),
        .package(path: "../scsdk"),
        .package(path: "../CppDependencies/json"),
        .package(path: "../CppDependencies/PoissonRecon"),
    ],
    targets: [
        .target(
            name: "StandardCyborgFusion",
            dependencies: [
                "json",
                "scsdk",
                "PoissonRecon",
                "ZipArchive",
            ],
            path: "Sources",
            // Eigen uses depth fusion, not the legacy ear/foot Vision classifiers.
            // Exclude their models to avoid Xcode 27 installing generated headers
            // into /usr/local/include for this Swift package target.
            exclude: ["StandardCyborgFusion/Models"],
            resources: [.process("Resources")],
            publicHeadersPath: "include",
            // Sensor depth uses NaN for missing observations. Preserve IEEE finite checks.
            cxxSettings: [
                // Always optimize, even for debug builds, in order to be usable while debugging the rest of an app
                .unsafeFlags(["-fobjc-arc", "-Os", "-fno-math-errno"]),
                .headerSearchPath("."),
                .headerSearchPath("../libigl/include"),
                .headerSearchPath("StandardCyborgFusion/Algorithm"),
                .headerSearchPath("StandardCyborgFusion/DataStructures"),
                .headerSearchPath("StandardCyborgFusion/EarLandmarking"),
                .headerSearchPath("StandardCyborgFusion/Helpers"),
                .headerSearchPath("StandardCyborgFusion/IO"),
                .headerSearchPath("StandardCyborgFusion/MetalDepthProcessor"),
                .headerSearchPath("StandardCyborgFusion/Private"),
                .headerSearchPath("include/StandardCyborgFusion"),
            ]
        ),
        .testTarget(
            name: "StandardCyborgFusionTests",
            dependencies: ["StandardCyborgFusion"],
            path: "Tests",
            resources: [
                .copy("StandardCyborgFusionTests/Data")
            ],
            // Sensor depth uses NaN for missing observations. Preserve IEEE finite checks.
            cxxSettings: [
                .define("DEBUG", .when(configuration: .debug)),
                .define("PROJECT_DIR", to: "\".\""),
                .unsafeFlags(["-fobjc-arc"]),
                .headerSearchPath("."),
                .headerSearchPath("../libigl/include"),
                .headerSearchPath("../Sources/StandardCyborgFusion/Algorithm"),
                .headerSearchPath("../Sources/StandardCyborgFusion/DataStructures"),
                .headerSearchPath("../Sources/StandardCyborgFusion/Helpers"),
                .headerSearchPath("../Sources/StandardCyborgFusion/IO"),
                .headerSearchPath("../Sources/StandardCyborgFusion/MetalDepthProcessor"),
                .headerSearchPath("../Sources/StandardCyborgFusion/Private"),
                .headerSearchPath("../Sources/include/StandardCyborgFusion"),
            ],
            linkerSettings: [
                .linkedFramework("XCTest"),
            ]
        )
    ],
    swiftLanguageModes: [.v5],
    cxxLanguageStandard: .cxx17
)
