using System;
using System.Globalization;
using System.IO;
using UnityEngine;

public class EnergyLogger : MonoBehaviour
{
    public bool writeCsv = true;
    public bool showOverlay = true;
    public string label = "Sim";
    public float overlayY = 70f;

    public Func<EnergySample> Sampler { get; set; }

    private StreamWriter csv;
    private string csvPath;
    private float simTime;
    private int frameNumber;
    private EnergySample latest;
    private GUIStyle style;

    private static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    void Start()
    {
        if (writeCsv) OpenCsv();
    }

    void OnDestroy() { CloseCsv(); }
    void OnApplicationQuit() { CloseCsv(); }

    public void Log(float dt)
    {
        latest = Sampler();
        frameNumber++;
        simTime += dt;
        if (csv != null)
            csv.WriteLine(string.Format(Inv,
                "{0};{1:F6};{2:F6};{3:F6};{4:F6};{5:F6}",
                frameNumber, simTime,
                latest.kinetic, latest.gravitational, latest.elastic, latest.Total));
    }

    void OnGUI()
    {
        if (!showOverlay) return;
        if (style == null)
        {
            style = new GUIStyle(GUI.skin.label) { fontSize = 14 };
            style.normal.textColor = Color.white;
        }
        GUI.Label(new Rect(10, overlayY, 700, 20),
            string.Format(Inv,
                "{0} energy  KE={1:F3}  PE_grav={2:F3}  PE_elastic={3:F3}  Total={4:F3}",
                label, latest.kinetic, latest.gravitational, latest.elastic, latest.Total),
            style);
    }

    private void OpenCsv()
    {
        var dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Data/EnergyLogs"));
        Directory.CreateDirectory(dir);
        string safe = string.IsNullOrEmpty(label) ? "Sim" : label;
        foreach (char c in Path.GetInvalidFileNameChars()) safe = safe.Replace(c, '_');
        csvPath = Path.Combine(dir, $"{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        csv = new StreamWriter(csvPath, false);
        csv.WriteLine("frame;time;kinetic;gravitational;elastic;total"); //energy in joules, time in seconds
        Debug.Log($"[EnergyLogger] {label}: writing energy log to {csvPath}");
    }

    private void CloseCsv()
    {
        if (csv == null) return;
        csv.Flush();
        csv.Close();
        csv = null;
    }
}
