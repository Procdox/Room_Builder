// Some copyright should be here...

using System.IO;
using UnrealBuildTool;

public class Room_Builder : ModuleRules
{
    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string ThirdPartyPath
    {
        get { return Path.GetFullPath(Path.Combine(ModulePath, "../../ThirdParty/")); }
    }

    public Room_Builder(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				"Room_Builder/Public"
				
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				"Room_Builder/Private",
				
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
                		"ProceduralMeshComponent"
				
				// ... add other public dependencies that you statically link with here ...
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",

				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);

        LoadClipper(Target);
    }

    public bool LoadClipper(ReadOnlyTargetRules Target)
    {
        bool isLibrarySupported = false;

        if ((Target.Platform == UnrealTargetPlatform.Win64) || (Target.Platform == UnrealTargetPlatform.Win32))
        {
            isLibrarySupported = true;

            string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64) ? "x64" : "x86";
            string LibrariesPath = Path.Combine(ThirdPartyPath, "ClipperLib", "Libraries");

            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "ClipperLib." + PlatformString + ".lib"));
        }

        if (isLibrarySupported)
        {
            PublicIncludePaths.Add(Path.Combine(ThirdPartyPath, "ClipperLib", "Includes"));
        }

        Definitions.Add(string.Format("WITH_CLIPPERLIB_BINDING={0}", isLibrarySupported ? 1 : 0));

        return isLibrarySupported;
    }
}
