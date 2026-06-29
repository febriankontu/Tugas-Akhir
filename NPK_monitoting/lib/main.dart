import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:firebase_core/firebase_core.dart';
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:maps_toolkit/maps_toolkit.dart' as mp;

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Smart Farming Lahan 5 (Baru)',
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xFF2E7D32), // Forest Green
          secondary: const Color(0xFF00796B), // Teal
          surface: const Color(0xFFF5F7F6),   // Broken White
        ),
        scaffoldBackgroundColor: const Color(0xFFF5F7F6),
        appBarTheme: const AppBarTheme(
          backgroundColor: Colors.white,
          elevation: 0,
          centerTitle: true,
          titleTextStyle: TextStyle(color: Color(0xFF1B5E20), fontWeight: FontWeight.bold, fontSize: 18),
          iconTheme: IconThemeData(color: Color(0xFF1B5E20)),
        ),
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
            padding: const EdgeInsets.symmetric(vertical: 12, horizontal: 24),
            textStyle: const TextStyle(fontWeight: FontWeight.bold, fontSize: 14),
          ),
        ),
      ),
      home: const AllInOnePage(),
    );
  }
}

class AllInOnePage extends StatefulWidget {
  const AllInOnePage({super.key});

  @override
  State<AllInOnePage> createState() => _AllInOnePageState();
}

class _AllInOnePageState extends State<AllInOnePage> {
  final FirebaseFirestore _firestore = FirebaseFirestore.instance;

  int _selectedIndex = 0;
  StreamSubscription<DocumentSnapshot>? _liveDataSub;
  Timer? _simulationTimer;

  // --- DATA NPK LAHAN 5 (Dari Tabel Distribusi) ---
  final List<Map<String, double>> _lahan5NpkDistribution = [
    {"N": 106, "P": 292, "K": 287}, // Titik 1
    {"N": 149, "P": 390, "K": 385}, // Titik 2
    {"N": 220, "P": 554, "K": 551}, // Titik 3
    {"N": 161, "P": 419, "K": 414}, // Titik 4
  ];

  // --- DATA LIVE ---
  // Default: Tengah Lahan 5 (Estimasi Rata-rata Baru)
  double? liveLat = -6.970652;
  double? liveLng = 107.630160;

  double liveN = 0, liveP = 0, liveK = 0;
  String statusGPS = "GPS Terhubung (Simulasi)";
  bool isGpsActive = true;

  // --- DATA LAHAN ---
  final List<Map<String, double>> _perimeterPoints = [];
  List<mp.LatLng> _targetGridPoints = [];
  Map<int, Map<String, double>> _measuredData = {};
  int? _selectedTargetIndex;

  // --- VARIABEL STATIS (Pengukuran) ---
  double _calculatedArea = 0.0;
  final double _fixedGridDistance = 2.0; // Grid Pengukuran 2m

  double _measureStepLat = 0.000018;
  double _measureStepLng = 0.000018;

  String _sessionStartTime = "";

  // --- ANALISIS (Heatmap High Res) ---
  List<double> _idwGridValues = [];
  bool _isAnalyzing = false;
  double _idwPower = 2.0;
  String _selectedNutrientMap = "N";

  // Grid halus untuk heatmap (0.4m)
  double _heatmapStepLat = 0.0000036;
  double _heatmapStepLng = 0.0000036;

  @override
  void initState() {
    super.initState();
    _sessionStartTime = DateTime.now().toIso8601String();

    // === KOORDINAT BARU LAHAN 5 ===
    _perimeterPoints.addAll([
      {"lat": -6.970637, "lng": 107.630174}, // Titik 1
      {"lat": -6.970640, "lng": 107.630149}, // Titik 2
      {"lat": -6.970664, "lng": 107.630144}, // Titik 3
      {"lat": -6.970667, "lng": 107.630171}, // Titik 4
    ]);

    _calculateGridSteps();
    _startListeningLive();
    _startSimulationTimer();
  }

  void _calculateGridSteps() {
    if (_perimeterPoints.isEmpty) return;
    double avgLat = 0;
    for(var p in _perimeterPoints) avgLat += p['lat']!;
    avgLat /= _perimeterPoints.length;

    // Step untuk grid pengukuran (2m)
    _measureStepLat = _fixedGridDistance / 111320.0;
    double cosLat = cos(avgLat * pi / 180.0);
    _measureStepLng = _fixedGridDistance / (111320.0 * cosLat);
  }

  @override
  void dispose() {
    _liveDataSub?.cancel();
    _simulationTimer?.cancel();
    super.dispose();
  }

  // === TIMER SIMULASI NPK RANDOM (Independen) ===
  void _startSimulationTimer() {
    _simulationTimer = Timer.periodic(const Duration(seconds: 2), (timer) {
      if (!mounted) return;
      final random = Random();
      int maxIdx = _lahan5NpkDistribution.length;

      // Ambil index acak terpisah untuk N, P, dan K
      int idxN = random.nextInt(maxIdx);
      int idxP = random.nextInt(maxIdx);
      int idxK = random.nextInt(maxIdx);

      setState(() {
        liveN = _lahan5NpkDistribution[idxN]["N"]!;
        liveP = _lahan5NpkDistribution[idxP]["P"]!;
        liveK = _lahan5NpkDistribution[idxK]["K"]!;
      });
    });
  }

  void _startListeningLive() {
    _liveDataSub = _firestore.collection('Data_testing').doc('live_data').snapshots().listen((snapshot) {
      if (!mounted) return;
      if (snapshot.exists && snapshot.data() != null) {
        var d = snapshot.data() as Map<String, dynamic>;
        double? realLat = (d['latitude'] is num) ? (d['latitude'] as num).toDouble() : null;
        double? realLng = (d['longitude'] is num) ? (d['longitude'] as num).toDouble() : null;

        setState(() {
          if (realLat != null && realLat != 0.0 && realLng != null && realLng != 0.0) {
            liveLat = realLat; liveLng = realLng; statusGPS = "GPS Terhubung";
          } else {
            // Fallback ke rata-rata koordinat baru
            liveLat = -6.970652; liveLng = 107.630160; statusGPS = "GPS Terhubung (Simulasi)";
          }
        });
      }
    });
  }

  Future<void> _rekamSudut() async {
    if (_perimeterPoints.length >= 4) {
      _showSnack("Sudah terisi 4 titik Lahan 5!", isError: true);
      return;
    }
    setState(() { _perimeterPoints.add({"lat": liveLat!, "lng": liveLng!}); });
    _calculateGridSteps();
  }

  void _generateGrid() {
    if (_perimeterPoints.length != 4) { _showSnack("Harus tepat 4 titik sudut!", isError: true); return; }

    _calculateGridSteps();

    List<mp.LatLng> polygon = _perimeterPoints.map((p) => mp.LatLng(p['lat']!, p['lng']!)).toList();
    double area = mp.SphericalUtil.computeArea(polygon).toDouble().abs();

    double minLat = polygon.first.latitude, maxLat = polygon.first.latitude;
    double minLng = polygon.first.longitude, maxLng = polygon.first.longitude;
    for (var p in polygon) {
      if (p.latitude < minLat) minLat = p.latitude; if (p.latitude > maxLat) maxLat = p.latitude;
      if (p.longitude < minLng) minLng = p.longitude; if (p.longitude > maxLng) maxLng = p.longitude;
    }

    // Grid Pengukuran (Resolusi 2m)
    List<mp.LatLng> tempGrid = [];
    for (double lat = minLat; lat < maxLat; lat += _measureStepLat) {
      for (double lng = minLng; lng < maxLng; lng += _measureStepLng) {
        mp.LatLng pCenter = mp.LatLng(lat + (_measureStepLat/2), lng + (_measureStepLng/2));
        if (mp.PolygonUtil.containsLocation(pCenter, polygon, false)) {
          tempGrid.add(pCenter);
        }
      }
    }

    if (tempGrid.isEmpty) {
      double cLat=0, cLng=0;
      for(var p in polygon){ cLat+=p.latitude; cLng+=p.longitude; }
      tempGrid.add(mp.LatLng(cLat/polygon.length, cLng/polygon.length));
    }

    tempGrid.sort((a, b) {
      double diffLat = a.latitude - b.latitude;
      if (diffLat.abs() > 0.000005) return a.latitude.compareTo(b.latitude);
      return a.longitude.compareTo(b.longitude);
    });

    setState(() {
      _targetGridPoints = tempGrid;
      _selectedTargetIndex = null;
      _measuredData.clear();
      _idwGridValues.clear();
      _calculatedArea = area;
    });
    _showSnack("Grid 2x2m: ${tempGrid.length} titik");
  }

  Future<void> _simpanPengukuran() async {
    if (_targetGridPoints.isEmpty || _selectedTargetIndex == null) return;
    int idx = _selectedTargetIndex!;
    String id = DateTime.now().toIso8601String();

    await _firestore.collection('Data_testing').doc(id).set({
      "latitude": _targetGridPoints[idx].latitude,
      "longitude": _targetGridPoints[idx].longitude,
      "nitrogen": liveN, "phosphorus": liveP, "potassium": liveK,
      "tipe": "riwayat", "waktu_simpan": id
    });

    setState(() {
      _measuredData[idx] = { "N": liveN, "P": liveP, "K": liveK, "lat": _targetGridPoints[idx].latitude, "lng": _targetGridPoints[idx].longitude };
      if (idx + 1 < _targetGridPoints.length) _selectedTargetIndex = idx + 1; else _selectedTargetIndex = null;
    });
    _showSnack("Data Tersimpan (Target ${idx+1})", isError: false);
  }

  Future<void> _saveSessionToHistory() async {
    if (_perimeterPoints.length != 4) { _showSnack("Batas lahan tidak valid!", isError: true); return; }
    TextEditingController nameCtrl = TextEditingController();
    await showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text("Simpan Sesi ke Histori"),
        content: TextField(
          controller: nameCtrl,
          decoration: const InputDecoration(hintText: "Nama Lahan (Contoh: Lahan 5 Baru)", border: OutlineInputBorder()),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context), child: const Text("Batal")),
          ElevatedButton(
            onPressed: () async {
              if (nameCtrl.text.isEmpty) return;
              Navigator.pop(context);

              List<Map<String,dynamic>> measures = [];
              _measuredData.forEach((key, val) {
                measures.add({ "index": key, "N": val["N"], "P": val["P"], "K": val["K"], "lat": val["lat"], "lng": val["lng"] });
              });

              await _firestore.collection('farm_history').add({
                "nama": nameCtrl.text,
                "tanggal": DateTime.now().toIso8601String(),
                "luas_area": _calculatedArea,
                "jarak_grid": _fixedGridDistance,
                "perimeter": _perimeterPoints,
                "measurements": measures,
              });

              _showSnack("Berhasil Disimpan!", isError: false);
              _resetLocalSession();
            },
            child: const Text("SIMPAN"),
          )
        ],
      ),
    );
  }

  void _resetLocalSession() {
    setState(() {
      _perimeterPoints.clear();
      _targetGridPoints.clear();
      _measuredData.clear();
      _idwGridValues.clear();
      _selectedTargetIndex = null;
      _calculatedArea = 0.0;
    });
  }

  // === GENERATE HEATMAP (SMOOTHER: RESOLUSI 0.4m) ===
  Future<void> _processIDW() async {
    setState(() => _isAnalyzing = true);
    if (_measuredData.isEmpty) { setState(() => _isAnalyzing = false); return; }

    List<mp.LatLng> samples = []; List<double> values = [];
    _measuredData.forEach((idx, d) {
      samples.add(mp.LatLng(d['lat']!, d['lng']!));
      double val = 0;
      if (_selectedNutrientMap == "N") val = d['N']!;
      else if (_selectedNutrientMap == "P") val = d['P']!;
      else if (_selectedNutrientMap == "K") val = d['K']!;
      values.add(val);
    });

    // Step grid diperkecil (0.4m) agar pixel lebih rapat dan gradasi halus
    double heatmapResolution = 0.4;
    double avgLat = _perimeterPoints[0]['lat']!;
    _heatmapStepLat = heatmapResolution / 111320.0;
    _heatmapStepLng = heatmapResolution / (111320.0 * cos(avgLat * pi / 180.0));

    List<double> results = _calculateIDWLogic(samples, values, _perimeterPoints, _heatmapStepLat, _heatmapStepLng);
    setState(() { _idwGridValues = results; _isAnalyzing = false; });
  }

  List<double> _calculateIDWLogic(List<mp.LatLng> samples, List<double> values, List<Map<String, double>> perimeter, double stepLat, double stepLng) {
    List<mp.LatLng> poly = perimeter.map((p) => mp.LatLng(p['lat']!, p['lng']!)).toList();
    double minLat=poly[0].latitude, maxLat=poly[0].latitude, minLng=poly[0].longitude, maxLng=poly[0].longitude;
    for (var p in poly) {
      if(p.latitude<minLat)minLat=p.latitude; if(p.latitude>maxLat)maxLat=p.latitude;
      if(p.longitude<minLng)minLng=p.longitude; if(p.longitude>maxLng)maxLng=p.longitude;
    }

    double pad = 0.00001;
    double gridMinLat = minLat - pad;
    double gridMaxLat = maxLat + pad;
    double gridMinLng = minLng - pad;
    double gridMaxLng = maxLng + pad;

    List<double> results = [];
    for (double lat = gridMinLat; lat < gridMaxLat; lat += stepLat) {
      for (double lng = gridMinLng; lng < gridMaxLng; lng += stepLng) {
        mp.LatLng gridP = mp.LatLng(lat + (stepLat/2), lng + (stepLng/2));
        double num=0, den=0;
        for (int k=0; k<samples.length; k++) {
          double dist = mp.SphericalUtil.computeDistanceBetween(gridP, samples[k]).toDouble();
          if(dist==0) dist=0.001;
          double w = 1/pow(dist, _idwPower);
          num += w*values[k]; den += w;
        }
        results.add(den==0 ? 0.0 : num/den);
      }
    }
    return results;
  }

  void _showSnack(String msg, {bool isError = false}) {
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(
      content: Text(msg),
      backgroundColor: isError ? Colors.redAccent : Colors.green,
      behavior: SnackBarBehavior.floating,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
      margin: const EdgeInsets.all(10),
    ));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text("SMART FARMING - LAHAN 5")),
      body: IndexedStack(index: _selectedIndex, children: [_buildPlanTab(), _buildMeasureTab(), _buildAnalyzeTab(), _buildHistoryTab()]),
      bottomNavigationBar: NavigationBarTheme(
        data: NavigationBarThemeData(
          indicatorColor: Theme.of(context).colorScheme.secondary.withOpacity(0.2),
          labelTextStyle: MaterialStateProperty.all(const TextStyle(fontSize: 12, fontWeight: FontWeight.bold)),
        ),
        child: NavigationBar(
          selectedIndex: _selectedIndex,
          onDestinationSelected: (i) => setState(() => _selectedIndex = i),
          destinations: const [
            NavigationDestination(icon: Icon(Icons.architecture_rounded), label: 'Rencana'),
            NavigationDestination(icon: Icon(Icons.speed_rounded), label: 'Ukur'),
            NavigationDestination(icon: Icon(Icons.map_rounded), label: 'Analisis'),
            NavigationDestination(icon: Icon(Icons.history_rounded), label: 'Riwayat'),
          ],
        ),
      ),
    );
  }

  Widget _coordItem(String label, String val) {
    return Column(
      children: [
        Text(label.toUpperCase(), style: TextStyle(fontSize: 10, color: Colors.grey[600])),
        Text(val, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600, fontFamily: 'monospace')),
      ],
    );
  }

  Widget _infoCard(String title, String value, IconData icon) {
    return Column(
      children: [
        Icon(icon, size: 20, color: Theme.of(context).colorScheme.secondary),
        const SizedBox(height: 4),
        Text(title, style: TextStyle(fontSize: 10, color: Colors.grey[600])),
        Text(value, style: const TextStyle(fontSize: 14, fontWeight: FontWeight.bold, color: Colors.black87)),
      ],
    );
  }

  // === TAB 1: RENCANA ===
  Widget _buildPlanTab() {
    return Column(
      children: [
        Container(
          margin: const EdgeInsets.all(16),
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(16), boxShadow: [BoxShadow(color: Colors.black12, blurRadius: 10)]),
          child: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      color: isGpsActive ? Colors.green[50] : Colors.red[50],
                      borderRadius: BorderRadius.circular(20),
                      border: Border.all(color: isGpsActive ? Colors.green : Colors.red),
                    ),
                    child: Row(
                      children: [
                        Icon(isGpsActive ? Icons.gps_fixed : Icons.gps_off, size: 16, color: isGpsActive ? Colors.green : Colors.red),
                        const SizedBox(width: 8),
                        Text(statusGPS, style: TextStyle(color: isGpsActive ? Colors.green[800] : Colors.red[800], fontWeight: FontWeight.bold, fontSize: 12)),
                      ],
                    ),
                  ),
                  if (_targetGridPoints.isNotEmpty)
                    Container(padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6), decoration: BoxDecoration(color: Colors.blue[50], borderRadius: BorderRadius.circular(20), border: Border.all(color: Colors.blue)), child: Text("${_targetGridPoints.length} Titik", style: TextStyle(color: Colors.blue[800], fontWeight: FontWeight.bold, fontSize: 12))),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceAround,
                children: [
                  _infoCard("Luas Lahan", "${_calculatedArea.toStringAsFixed(2)} m²", Icons.landscape),
                  _infoCard("Jarak Grid", "${_fixedGridDistance} m", Icons.straighten),
                ],
              ),
              const Divider(),
              if (liveLat != null)
                Row(mainAxisAlignment: MainAxisAlignment.spaceAround, children: [_coordItem("Latitude", liveLat!.toStringAsFixed(6)), Container(width: 1, height: 30, color: Colors.grey[300]), _coordItem("Longitude", liveLng!.toStringAsFixed(6))])
              else
                const Text("Menunggu Koordinat...", style: TextStyle(color: Colors.grey, fontStyle: FontStyle.italic)),
            ],
          ),
        ),
        Expanded(
          child: Container(
            margin: const EdgeInsets.symmetric(horizontal: 16),
            decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(16), boxShadow: [BoxShadow(color: Colors.black12, blurRadius: 5)]),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(16),
              child: Stack(
                children: [
                  CustomPaint(
                    painter: FieldPainter(
                        perimeter: _perimeterPoints,
                        gridPoints: _targetGridPoints,
                        userLat: liveLat, userLng: liveLng,
                        showCornerNumbers: true,
                        stepLat: _measureStepLat, // Grid kasar 2m untuk visual Rencana
                        stepLng: _measureStepLng,
                        showGridDots: true
                    ),
                    child: Container(width: double.infinity, height: double.infinity),
                  ),
                  Positioned(
                    bottom: 16, right: 16,
                    child: FloatingActionButton.extended(
                      onPressed: _rekamSudut,
                      icon: const Icon(Icons.add_location_alt_outlined),
                      label: const Text("Rekam"),
                      backgroundColor: _perimeterPoints.length >= 4 ? Colors.grey : Theme.of(context).colorScheme.secondary,
                      foregroundColor: Colors.white,
                    ),
                  )
                ],
              ),
            ),
          ),
        ),
        Padding(
          padding: const EdgeInsets.all(16),
          child: Row(
            children: [
              Expanded(child: ElevatedButton(onPressed: _targetGridPoints.isEmpty ? (_perimeterPoints.length != 4 ? null : _generateGrid) : null, style: ElevatedButton.styleFrom(backgroundColor: Theme.of(context).colorScheme.primary, foregroundColor: Colors.white), child: const Text("HITUNG GRID"))),
              const SizedBox(width: 8),
              Expanded(child: ElevatedButton(onPressed: _targetGridPoints.isEmpty ? null : _saveSessionToHistory, style: ElevatedButton.styleFrom(backgroundColor: Colors.orange[700], foregroundColor: Colors.white), child: const Text("SIMPAN SESI"))),
              const SizedBox(width: 8),
              SizedBox(width: 40, child: OutlinedButton(onPressed: _resetLocalSession, style: OutlinedButton.styleFrom(padding: EdgeInsets.zero, foregroundColor: Colors.red, side: const BorderSide(color: Colors.red)), child: const Icon(Icons.delete_outline, size: 20))),
            ],
          ),
        )
      ],
    );
  }

  // === TAB 2: UKUR ===
  Widget _buildMeasureTab() {
    return Column(
      children: [
        Expanded(
          flex: 4,
          child: Container(
            margin: const EdgeInsets.all(16),
            decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(16), border: Border.all(color: Colors.grey.shade300)),
            child: ClipRRect(
              borderRadius: BorderRadius.circular(16),
              child: CustomPaint(
                painter: FieldPainter(
                    perimeter: _perimeterPoints,
                    gridPoints: _targetGridPoints,
                    userLat: liveLat, userLng: liveLng,
                    showCornerNumbers: false,
                    stepLat: _measureStepLat,
                    stepLng: _measureStepLng,
                    showGridDots: true,
                    highlightIndex: _selectedTargetIndex
                ),
                child: Container(width: double.infinity, height: double.infinity),
              ),
            ),
          ),
        ),
        Expanded(
          flex: 6,
          child: Container(
            decoration: const BoxDecoration(color: Colors.white, borderRadius: BorderRadius.vertical(top: Radius.circular(24)), boxShadow: [BoxShadow(color: Colors.black12, blurRadius: 10, offset: Offset(0, -5))]),
            child: Column(
              children: [
                const SizedBox(height: 8), Container(width: 40, height: 4, decoration: BoxDecoration(color: Colors.grey[300], borderRadius: BorderRadius.circular(2))),
                Padding(padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 24), child: Row(mainAxisAlignment: MainAxisAlignment.spaceBetween, children: [_sensorBox("N", liveN, Colors.blue), _sensorBox("P", liveP, Colors.orange), _sensorBox("K", liveK, Colors.purple)])),
                Padding(padding: const EdgeInsets.symmetric(horizontal: 24), child: SizedBox(width: double.infinity, child: ElevatedButton.icon(onPressed: _simpanPengukuran, style: ElevatedButton.styleFrom(backgroundColor: _selectedTargetIndex == null ? Colors.grey : Theme.of(context).colorScheme.primary, foregroundColor: Colors.white), icon: const Icon(Icons.save_rounded), label: Text(_selectedTargetIndex == null ? "PILIH TITIK TARGET" : "SIMPAN DATA")))),
                const Padding(padding: EdgeInsets.only(left: 24, right: 24, top: 12), child: Divider()),
                Expanded(
                  child: ListView.builder(
                    padding: const EdgeInsets.symmetric(horizontal: 16), itemCount: _targetGridPoints.length,
                    itemBuilder: (c, i) {
                      bool hasData = _measuredData.containsKey(i); bool isSel = (i == _selectedTargetIndex);
                      return Container(
                        margin: const EdgeInsets.only(bottom: 8),
                        decoration: BoxDecoration(color: isSel ? Colors.green[50] : (hasData ? Colors.white : Colors.grey[50]), borderRadius: BorderRadius.circular(12), border: Border.all(color: isSel ? Colors.green : Colors.transparent, width: isSel ? 2 : 1)),
                        child: ListTile(
                          onTap: () => setState(() => _selectedTargetIndex = i),
                          leading: CircleAvatar(backgroundColor: hasData ? Colors.green : (isSel ? Theme.of(context).colorScheme.primary : Colors.grey[300]), foregroundColor: Colors.white, child: hasData ? const Icon(Icons.check, size: 16) : Text("${i+1}", style: const TextStyle(fontWeight: FontWeight.bold))),
                          title: Text("Target ${i+1}", style: TextStyle(fontWeight: FontWeight.bold, color: isSel ? Colors.green[800] : Colors.black87)),
                          trailing: hasData ? Row(mainAxisSize: MainAxisSize.min, children: [_miniTag("N", _measuredData[i]?['N'], Colors.blue), const SizedBox(width: 4), _miniTag("P", _measuredData[i]?['P'], Colors.orange), const SizedBox(width: 4), _miniTag("K", _measuredData[i]?['K'], Colors.purple)]) : const Icon(Icons.chevron_right, color: Colors.grey),
                        ),
                      );
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _sensorBox(String label, double val, Color color) {
    return Column(children: [Container(width: 60, height: 60, alignment: Alignment.center, decoration: BoxDecoration(color: color.withOpacity(0.1), borderRadius: BorderRadius.circular(12), border: Border.all(color: color.withOpacity(0.3))), child: Text("${val.toInt()}", style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: color))), const SizedBox(height: 4), Text(label, style: TextStyle(fontWeight: FontWeight.bold, color: color))]);
  }
  Widget _miniTag(String l, double? v, Color c) => Container(padding: const EdgeInsets.symmetric(horizontal: 6, vertical: 2), decoration: BoxDecoration(color: c.withOpacity(0.1), borderRadius: BorderRadius.circular(4)), child: Text("$l:${v?.toInt()}", style: TextStyle(fontSize: 10, fontWeight: FontWeight.bold, color: c)));

  // === TAB 3: ANALISIS (Heatmap High Res 0.4m) ===
  Widget _buildAnalyzeTab() {
    return Column(
      children: [
        Container(padding: const EdgeInsets.symmetric(vertical: 16, horizontal: 16), color: Colors.white, child: Row(mainAxisAlignment: MainAxisAlignment.spaceEvenly, children: [_mapOption("Nitrogen (N)", "N", Colors.blue), _mapOption("Fosfor (P)", "P", Colors.orange), _mapOption("Kalium (K)", "K", Colors.purple)])),
        Expanded(child: Container(margin: const EdgeInsets.all(16), decoration: BoxDecoration(color: Colors.white, border: Border.all(color: Colors.grey.shade300), borderRadius: BorderRadius.circular(16)), child: _idwGridValues.isEmpty ? const Center(child: Text("Belum ada data generated.")) : ClipRRect(borderRadius: BorderRadius.circular(16), child: Stack(children: [
          CustomPaint(
              painter: HeatmapPainter(
                  values: _idwGridValues,
                  perimeter: _perimeterPoints,
                  stepLat: _heatmapStepLat, // Gunakan grid halus
                  stepLng: _heatmapStepLng,
                  nutrientType: _selectedNutrientMap
              ),
              child: Container(width: double.infinity, height: double.infinity)
          ),
          CustomPaint(painter: FieldPainter(perimeter: _perimeterPoints, gridPoints: [], userLat: null, userLng: null, showGridDots: false, showGridLines: false, onlyBorder: true), child: Container(width: double.infinity, height: double.infinity))])))),
        Container(padding: const EdgeInsets.all(16), color: Colors.white, child: Column(children: [Row(mainAxisAlignment: MainAxisAlignment.center, children: [_legendItem(Colors.red, "Rendah"), const SizedBox(width: 24), _legendItem(Colors.yellow, "Sedang"), const SizedBox(width: 24), _legendItem(Colors.green, "Tinggi")]), const SizedBox(height: 16), SizedBox(width: double.infinity, child: ElevatedButton.icon(onPressed: (_isAnalyzing || _sessionStartTime=="") ? null : _processIDW, style: ElevatedButton.styleFrom(backgroundColor: const Color(0xFF3F51B5), foregroundColor: Colors.white), icon: _isAnalyzing ? const SizedBox(width: 20, height: 20, child: CircularProgressIndicator(color: Colors.white, strokeWidth: 2)) : const Icon(Icons.analytics_rounded), label: Text(_isAnalyzing ? "SEDANG MEMPROSES..." : "GENERATE PETA SEBARAN")))]))
      ],
    );
  }
  Widget _mapOption(String label, String code, Color color) { bool isSel = _selectedNutrientMap == code; return GestureDetector(onTap: (){ setState(()=>_selectedNutrientMap=code); if(_idwGridValues.isNotEmpty)_processIDW(); }, child: AnimatedContainer(duration: const Duration(milliseconds: 200), padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8), decoration: BoxDecoration(color: isSel ? color : Colors.white, borderRadius: BorderRadius.circular(20), border: Border.all(color: color, width: 1.5), boxShadow: isSel ? [BoxShadow(color: color.withOpacity(0.4), blurRadius: 6, offset: const Offset(0, 3))] : []), child: Text(label, style: TextStyle(color: isSel ? Colors.white : color, fontWeight: FontWeight.bold, fontSize: 12)))); }
  Widget _legendItem(Color c, String t) => Row(children: [Container(width: 12, height: 12, decoration: BoxDecoration(color: c, shape: BoxShape.circle)), const SizedBox(width: 6), Text(t, style: const TextStyle(fontSize: 12, fontWeight: FontWeight.w600))]);

  // === TAB 4: RIWAYAT ===
  Widget _buildHistoryTab() {
    return StreamBuilder<QuerySnapshot>(
      stream: _firestore.collection('farm_history').orderBy('tanggal', descending: true).snapshots(),
      builder: (context, snapshot) {
        if (!snapshot.hasData) return const Center(child: CircularProgressIndicator());
        var docs = snapshot.data!.docs;
        if (docs.isEmpty) return const Center(child: Text("Belum ada riwayat tersimpan."));
        return ListView.builder(
          padding: const EdgeInsets.all(16), itemCount: docs.length,
          itemBuilder: (context, index) {
            var data = docs[index].data() as Map<String, dynamic>;
            String nama = data['nama'] ?? "Tanpa Nama";
            String tgl = data['tanggal'] ?? "";
            double area = (data['luas_area'] is num) ? (data['luas_area'] as num).toDouble() : 0.0;
            double gridDist = (data['jarak_grid'] is num) ? (data['jarak_grid'] as num).toDouble() : 0.0;
            String formattedDate = "-";
            if (tgl.isNotEmpty) { DateTime dt = DateTime.parse(tgl); formattedDate = "${dt.day}/${dt.month}/${dt.year}"; }
            return Card(
              margin: const EdgeInsets.only(bottom: 12),
              child: ListTile(
                leading: CircleAvatar(backgroundColor: Colors.green[100], child: const Icon(Icons.history_edu, color: Colors.green)),
                title: Text(nama, style: const TextStyle(fontWeight: FontWeight.bold)),
                subtitle: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [Text(formattedDate), if(area > 0) Text("Luas: ${area.toStringAsFixed(1)} m² | Grid: ${gridDist}m", style: TextStyle(fontSize: 10, color: Colors.grey[700]))]),
                trailing: const Icon(Icons.arrow_forward_ios, size: 16),
                onTap: () { Navigator.push(context, MaterialPageRoute(builder: (c) => HistoryDetailPage(data: data))); },
              ),
            );
          },
        );
      },
    );
  }
}

// === PAGE BARU: DETAIL HISTORY ===
class HistoryDetailPage extends StatefulWidget {
  final Map<String, dynamic> data;
  const HistoryDetailPage({super.key, required this.data});
  @override State<HistoryDetailPage> createState() => _HistoryDetailPageState();
}
class _HistoryDetailPageState extends State<HistoryDetailPage> {
  List<Map<String, double>> perimeter = []; List<double> idwValues = []; String selectedNutrient = "N";
  double stepLat = 0.000018; double stepLng = 0.000018;

  @override void initState() { super.initState();
  List rawPerim = widget.data['perimeter'] ?? [];
  perimeter = rawPerim.map((e) => {"lat": (e['lat'] as num).toDouble(), "lng": (e['lng'] as num).toDouble()}).toList();
  _recalcSteps();
  _processHistoryIDW();
  }

  void _recalcSteps() {
    if (perimeter.isEmpty) return;
    double avgLat = 0;
    for(var p in perimeter) avgLat+=p['lat']!;
    avgLat/=perimeter.length;

    // Default 0.4m untuk history juga biar smooth
    double dist = 0.4;
    stepLat = dist/111320.0;
    stepLng = dist/(111320.0 * cos(avgLat*pi/180.0));
  }

  void _processHistoryIDW() {
    List rawMeas = widget.data['measurements'] ?? []; List<mp.LatLng> samples = []; List<double> values = [];
    for (var m in rawMeas) { samples.add(mp.LatLng((m['lat'] as num).toDouble(), (m['lng'] as num).toDouble())); double val = 0; if (selectedNutrient == "N") val = (m['N'] as num).toDouble(); else if (selectedNutrient == "P") val = (m['P'] as num).toDouble(); else if (selectedNutrient == "K") val = (m['K'] as num).toDouble(); values.add(val); }
    if (samples.isEmpty || perimeter.isEmpty) return;

    List<mp.LatLng> poly = perimeter.map((p) => mp.LatLng(p['lat']!, p['lng']!)).toList();
    double minLat=poly[0].latitude, maxLat=poly[0].latitude, minLng=poly[0].longitude, maxLng=poly[0].longitude;
    for (var p in poly) { if(p.latitude<minLat)minLat=p.latitude; if(p.latitude>maxLat)maxLat=p.latitude; if(p.longitude<minLng)minLng=p.longitude; if(p.longitude>maxLng)maxLng=p.longitude;}
    double pad = 0.00001; double gridMinLat = minLat - pad; double gridMaxLat = maxLat + pad; double gridMinLng = minLng - pad; double gridMaxLng = maxLng + pad;

    List<double> results = [];
    for (double lat = gridMinLat; lat < gridMaxLat; lat += stepLat) {
      for (double lng = gridMinLng; lng < gridMaxLng; lng += stepLng) {
        mp.LatLng gridP = mp.LatLng(lat + (stepLat/2), lng + (stepLng/2));
        double num=0, den=0;
        for (int k=0; k<samples.length; k++) { double dist = mp.SphericalUtil.computeDistanceBetween(gridP, samples[k]).toDouble(); if(dist==0) dist=0.001; double w = 1/pow(dist, 2.0); num += w*values[k]; den += w; }
        results.add(den==0 ? 0.0 : num/den);
      }
    }
    setState(() { idwValues = results; });
  }

  @override Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text(widget.data['nama'] ?? "Detail")),
      body: Column(children: [
        Container(padding: const EdgeInsets.all(16), color: Colors.white, child: Row(mainAxisAlignment: MainAxisAlignment.spaceEvenly, children: [_btn("N", Colors.blue), _btn("P", Colors.orange), _btn("K", Colors.purple)])),
        Expanded(child: Container(margin: const EdgeInsets.all(16), decoration: BoxDecoration(border: Border.all(color: Colors.grey), color: Colors.white), child: ClipRRect(child: Stack(children: [
          CustomPaint(
              painter: HeatmapPainter(
                  values: idwValues,
                  perimeter: perimeter,
                  stepLat: stepLat, stepLng: stepLng,
                  nutrientType: selectedNutrient
              ),
              child: Container()
          ),
          CustomPaint(painter: FieldPainter(perimeter: perimeter, gridPoints: [], onlyBorder: true), child: Container())])))),
        Padding(padding: const EdgeInsets.all(16), child: Row(mainAxisAlignment: MainAxisAlignment.center, children: [_leg(Colors.red,"Rendah"), const SizedBox(width:20), _leg(Colors.yellow,"Sedang"), const SizedBox(width:20), _leg(Colors.green,"Tinggi")]))
      ]),
    );
  }
  Widget _btn(String l, Color c) => GestureDetector(onTap: (){ setState(()=>selectedNutrient=l); _processHistoryIDW();}, child: Container(padding: const EdgeInsets.symmetric(horizontal:20, vertical:8), decoration: BoxDecoration(color: selectedNutrient==l?c:Colors.white, border: Border.all(color: c), borderRadius: BorderRadius.circular(20)), child: Text(l, style: TextStyle(color: selectedNutrient==l?Colors.white:c, fontWeight: FontWeight.bold))));
  Widget _leg(Color c, String t) => Row(children:[Container(width:12, height:12, color:c), const SizedBox(width:5), Text(t)]);
}

// === PAINTER REVISI (Menjaga Aspect Ratio) ===
class FieldPainter extends CustomPainter {
  final List<Map<String, double>> perimeter; final List<mp.LatLng> gridPoints; final double? userLat; final double? userLng; final bool showCornerNumbers; final double stepLat; final double stepLng; final bool showGridLines; final bool showGridDots; final bool onlyBorder; final int? highlightIndex;
  FieldPainter({required this.perimeter, required this.gridPoints, this.userLat, this.userLng, this.showCornerNumbers = false, this.stepLat=0, this.stepLng=0, this.showGridLines = true, this.showGridDots = true, this.onlyBorder = false, this.highlightIndex});
  @override
  void paint(Canvas canvas, Size size) {
    if (perimeter.isEmpty) return;
    double minLat = perimeter[0]['lat']!, maxLat = perimeter[0]['lat']!, minLng = perimeter[0]['lng']!, maxLng = perimeter[0]['lng']!;
    for (var p in perimeter) { if (p['lat']! < minLat) minLat = p['lat']!; if (p['lat']! > maxLat) maxLat = p['lat']!; if (p['lng']! < minLng) minLng = p['lng']!; if (p['lng']! > maxLng) maxLng = p['lng']!; }

    // --- LOGIKA ASPECT RATIO YANG DIPERBAIKI ---
    double midLat = (minLat+maxLat)/2;
    double heightMeters = (maxLat-minLat) * 111320;
    double widthMeters = (maxLng-minLng) * 111320 * cos(midLat*pi/180);

    if(heightMeters == 0) heightMeters = 1; if(widthMeters == 0) widthMeters = 1;

    double scaleX = size.width / widthMeters;
    double scaleY = size.height / heightMeters;
    double scale = min(scaleX, scaleY) * 0.9; // Margin 10%

    double drawWidth = widthMeters * scale;
    double drawHeight = heightMeters * scale;
    double offsetX = (size.width - drawWidth) / 2;
    double offsetY = (size.height - drawHeight) / 2;

    Offset toScreen(double lat, double lng) {
      double yM = (maxLat - lat) * 111320;
      double xM = (lng - minLng) * 111320 * cos(midLat*pi/180);
      return Offset(offsetX + (xM * scale), offsetY + (yM * scale));
    }

    Paint paintBorder = Paint()..color = const Color(0xFF2E7D32)..strokeWidth = 3.0..style = PaintingStyle.stroke..strokeJoin = StrokeJoin.round;
    Paint paintFill = Paint()..color = const Color(0xFF2E7D32).withOpacity(0.1)..style = PaintingStyle.fill;
    Paint paintGridLine = Paint()..color = Colors.grey.withOpacity(0.3)..strokeWidth = 1.0..style = PaintingStyle.stroke;

    if (!onlyBorder && showGridLines && stepLat > 0 && stepLng > 0) {
      for (double lng = minLng; lng <= maxLng; lng += stepLng) canvas.drawLine(toScreen(minLat, lng), toScreen(maxLat, lng), paintGridLine);
      for (double lat = minLat; lat <= maxLat; lat += stepLat) canvas.drawLine(toScreen(lat, minLng), toScreen(lat, maxLng), paintGridLine);
    }

    Path path = Path(); Offset start = toScreen(perimeter[0]['lat']!, perimeter[0]['lng']!); path.moveTo(start.dx, start.dy);
    for (int i = 1; i < perimeter.length; i++) { Offset p = toScreen(perimeter[i]['lat']!, perimeter[i]['lng']!); path.lineTo(p.dx, p.dy); } path.close();
    if (!onlyBorder) canvas.drawPath(path, paintFill); canvas.drawPath(path, paintBorder);
    if (onlyBorder) return;

    if (showCornerNumbers) {
      for (int i = 0; i < perimeter.length; i++) {
        Offset p = toScreen(perimeter[i]['lat']!, perimeter[i]['lng']!);
        canvas.drawCircle(p, 10, Paint()..color = Colors.orange);
        TextPainter tp = TextPainter(text: TextSpan(text: "${i+1}", style: const TextStyle(color: Colors.white, fontSize: 10, fontWeight: FontWeight.bold)), textDirection: TextDirection.ltr);
        tp.layout(); tp.paint(canvas, Offset(p.dx - tp.width/2, p.dy - tp.height/2));
      }
    }

    if (showGridDots) {
      for (int i = 0; i < gridPoints.length; i++) {
        Offset p = toScreen(gridPoints[i].latitude, gridPoints[i].longitude);
        if (highlightIndex != null && i == highlightIndex) { canvas.drawCircle(p, 12, Paint()..color = Colors.green.withOpacity(0.3)); canvas.drawCircle(p, 6, Paint()..color = Colors.green); } else { canvas.drawCircle(p, 3, Paint()..color = const Color(0xFF2E7D32)); }
      }
    }
    if (userLat != null && userLng != null) { Offset pUser = toScreen(userLat!, userLng!); canvas.drawCircle(pUser, 8, Paint()..color = Colors.red.withOpacity(0.3)); canvas.drawCircle(pUser, 5, Paint()..color = Colors.red); }
  }
  @override bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}

// === HEATMAP PAINTER (SOLID - NO GRID LINES) ===
class HeatmapPainter extends CustomPainter {
  final List<double> values; final List<Map<String, double>> perimeter; final double stepLat; final double stepLng; final String nutrientType;
  HeatmapPainter({required this.values, required this.perimeter, required this.stepLat, required this.stepLng, required this.nutrientType});
  @override
  void paint(Canvas canvas, Size size) {
    if (values.isEmpty || perimeter.isEmpty) return;
    double minLat = perimeter[0]['lat']!, maxLat = perimeter[0]['lat']!, minLng = perimeter[0]['lng']!, maxLng = perimeter[0]['lng']!;
    for (var p in perimeter) { if (p['lat']! < minLat) minLat = p['lat']!; if (p['lat']! > maxLat) maxLat = p['lat']!; if (p['lng']! < minLng) minLng = p['lng']!; if (p['lng']! > maxLng) maxLng = p['lng']!; }

    // --- LOGIKA ASPECT RATIO ---
    double midLat = (minLat+maxLat)/2;
    double heightMeters = (maxLat-minLat) * 111320;
    double widthMeters = (maxLng-minLng) * 111320 * cos(midLat*pi/180);
    if(heightMeters == 0) heightMeters = 1; if(widthMeters == 0) widthMeters = 1;
    double scaleX = size.width / widthMeters;
    double scaleY = size.height / heightMeters;
    double scale = min(scaleX, scaleY) * 0.9;
    double offsetX = (size.width - (widthMeters * scale)) / 2;
    double offsetY = (size.height - (heightMeters * scale)) / 2;

    Offset toScreen(double lat, double lng) {
      double yM = (maxLat - lat) * 111320;
      double xM = (lng - minLng) * 111320 * cos(midLat*pi/180);
      return Offset(offsetX + (xM * scale), offsetY + (yM * scale));
    }

    Path clipPath = Path(); Offset start = toScreen(perimeter[0]['lat']!, perimeter[0]['lng']!); clipPath.moveTo(start.dx, start.dy);
    for (int i = 1; i < perimeter.length; i++) { Offset p = toScreen(perimeter[i]['lat']!, perimeter[i]['lng']!); clipPath.lineTo(p.dx, p.dy); } clipPath.close();
    canvas.save(); canvas.clipPath(clipPath);

    double pad = 0.00001; double gridMinLat = minLat - pad; double gridMaxLat = maxLat + pad; double gridMinLng = minLng - pad; double gridMaxLng = maxLng + pad;
    int vIndex = 0;

    for (double lat = gridMinLat; lat < gridMaxLat; lat += stepLat) {
      for (double lng = gridMinLng; lng < gridMaxLng; lng += stepLng) {
        if (vIndex >= values.length) break;
        double val = values[vIndex]; vIndex++;
        Color c = Colors.grey;
        if (nutrientType == "N") { if (val < 50) c = Color.lerp(Colors.red, Colors.orange, val/50)!; else if (val < 150) c = Color.lerp(Colors.orange, Colors.yellow, (val-50)/100)!; else c = Color.lerp(Colors.yellow, Colors.green, (val-150)/100)!; if (val > 250) c = Colors.green; }
        else if (nutrientType == "P") { if (val < 30) c = Color.lerp(Colors.red, Colors.orange, val/30)!; else if (val < 80) c = Color.lerp(Colors.orange, Colors.yellow, (val-30)/50)!; else c = Colors.green; }
        else if (nutrientType == "K") { if (val < 80) c = Color.lerp(Colors.red, Colors.orange, val/80)!; else if (val < 150) c = Color.lerp(Colors.orange, Colors.yellow, (val-80)/70)!; else c = Colors.green; }

        Offset pBottomLeft = toScreen(lat, lng);
        Offset pTopRight = toScreen(lat + stepLat, lng + stepLng);
        Rect rect = Rect.fromPoints(pBottomLeft, pTopRight);

        // HANYA FILL, TIDAK ADA STROKE (GRID LINE)
        canvas.drawRect(rect, Paint()..color = c..style = PaintingStyle.fill);
      }
    }
    canvas.restore();
  }
  @override bool shouldRepaint(covariant CustomPainter oldDelegate) => true;
}