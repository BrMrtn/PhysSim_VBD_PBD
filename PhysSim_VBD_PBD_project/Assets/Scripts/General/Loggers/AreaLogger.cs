using System;
using System.Globalization;
using System.IO;
using UnityEngine;

public class AreaLogger : MonoBehaviour
{
    public bool writeCsv = true;
    public bool showOverlay = true;
    public string label = "Sim";
    public float overlayY = 50f;

    public Func<AreaSample> Sampler { get; set; }

    private StreamWriter csv;
    private string csvPath;
    private float simTime;
    private int frameNumber;
    private AreaSample latest;
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
                "{0};{1:F6};{2:F6};{3:F6};{4:F6}",
                frameNumber, simTime,
                latest.current, latest.rest, latest.Ratio));
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
                "{0} area  current={1:F4}  rest={2:F4}  ratio={3:F4}x",
                label, latest.current, latest.rest, latest.Ratio),
            style);
    }

    private void OpenCsv()
    {
        var dir = Path.GetFullPath(Path.Combine(Application.dataPath, "../../Logs/AreaLogs"));
        Directory.CreateDirectory(dir);
        string safe = string.IsNullOrEmpty(label) ? "Sim" : label;
        foreach (char c in Path.GetInvalidFileNameChars()) safe = safe.Replace(c, '_');
        csvPath = Path.Combine(dir, $"{safe}_{DateTime.Now:yyyyMMdd_HHmmss}.csv");
        csv = new StreamWriter(csvPath, false);
        csv.WriteLine("frame;time;current;rest"); // area in m^2, time in seconds
        Debug.Log($"[AreaLogger] {label}: writing area log to {csvPath}");
    }

    private void CloseCsv()
    {
        if (csv == null) return;
        csv.Flush();
        csv.Close();
        csv = null;
    }
}
