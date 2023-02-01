var app = require('express')();
var server = require('http');
const bodyParser = require('body-parser');
const multer  = require('multer');
const fs = require('fs');


const upload = multer({ dest: 'uploads' });


app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: false}));
app.use((req, res, next) => {
  res.setHeader("Access-Control-Allow-Origin", "*");
  res.setHeader(
    "Access-Control-Allow-Headers",
    "Origin, X-Requested-With, Content-Type, Accept"
  );
  res.setHeader(
    "Access-Control-Allow-Methods",
    "GET, POST, PUT, DELETE, OPTIONS"
  );
  next();
});

app.get('/', function (req, res) {
	res.status(200).json("Hello world!");
})
.post('/upload', upload.single('part'), function(req, res) {
  const title = req.body.title;
  const file = req.file;

  console.log(title);
  console.log(file);

  res.status(200).json({data: file, message: 'success'});
})
.post('/json', function(req, res) {
  const jsonFile = req.body;
  const filename = "/var/path.json";

  var json_string = JSON.stringify(jsonFile);
  fs.writeFileSync(filename, json_string);
 
  res.status(200).json({message:'success'});

})
.get('/download', function(req, res) {
  const file = '/var/path.json';
  res.download(file);
});

app.listen(5000, function () {
    console.log('Server is running..');
});
