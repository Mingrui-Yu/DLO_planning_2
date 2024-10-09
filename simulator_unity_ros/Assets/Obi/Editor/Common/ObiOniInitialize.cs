using System;
using System.Collections.Generic;
using UnityEditor;

namespace Obi
{
    [InitializeOnLoad]
    public class ObiOniInitialize
    {
        private static BuildTargetGroup[] supportedBuildTargetGroups =
        {
                BuildTargetGroup.Standalone,
                BuildTargetGroup.Android,
                BuildTargetGroup.iOS
        };

        static ObiOniInitialize()
        {

            foreach(var group in supportedBuildTargetGroups)
            {
                var defines = GetDefinesList(group);
                if (!defines.Contains("OBI_ONI_SUPPORTED"))
                {
                    defines.Add("OBI_ONI_SUPPORTED");
                    PlayerSettings.SetScriptingDefineSymbolsForGroup(group, string.Join(";", defines.ToArray()));
                }
            }
        }

        private static List<string> GetDefinesList(BuildTargetGroup group)
        {
            return new List<string>(PlayerSettings.GetScriptingDefineSymbolsForGroup(group).Split(';'));
        }
    }
}
