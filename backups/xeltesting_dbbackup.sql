USE [master]
GO
/****** Object:  Database [Ectobox]    Script Date: 11/20/2023 9:20:24 AM ******/
CREATE DATABASE [Ectobox]
 CONTAINMENT = NONE
 ON  PRIMARY 
( NAME = N'Ectobox', FILENAME = N'/var/opt/mssql/data/Ectobox.mdf' , SIZE = 8192KB , MAXSIZE = UNLIMITED, FILEGROWTH = 10%)
 LOG ON 
( NAME = N'Ectobox_log', FILENAME = N'/var/opt/mssql/data/Ectobox_log.ldf' , SIZE = 10944KB , MAXSIZE = 2048GB , FILEGROWTH = 10%)
 WITH CATALOG_COLLATION = DATABASE_DEFAULT
GO
ALTER DATABASE [Ectobox] SET COMPATIBILITY_LEVEL = 140
GO
IF (1 = FULLTEXTSERVICEPROPERTY('IsFullTextInstalled'))
begin
EXEC [Ectobox].[dbo].[sp_fulltext_database] @action = 'enable'
end
GO
ALTER DATABASE [Ectobox] SET ANSI_NULL_DEFAULT OFF 
GO
ALTER DATABASE [Ectobox] SET ANSI_NULLS OFF 
GO
ALTER DATABASE [Ectobox] SET ANSI_PADDING OFF 
GO
ALTER DATABASE [Ectobox] SET ANSI_WARNINGS OFF 
GO
ALTER DATABASE [Ectobox] SET ARITHABORT OFF 
GO
ALTER DATABASE [Ectobox] SET AUTO_CLOSE OFF 
GO
ALTER DATABASE [Ectobox] SET AUTO_SHRINK OFF 
GO
ALTER DATABASE [Ectobox] SET AUTO_UPDATE_STATISTICS ON 
GO
ALTER DATABASE [Ectobox] SET CURSOR_CLOSE_ON_COMMIT OFF 
GO
ALTER DATABASE [Ectobox] SET CURSOR_DEFAULT  GLOBAL 
GO
ALTER DATABASE [Ectobox] SET CONCAT_NULL_YIELDS_NULL OFF 
GO
ALTER DATABASE [Ectobox] SET NUMERIC_ROUNDABORT OFF 
GO
ALTER DATABASE [Ectobox] SET QUOTED_IDENTIFIER OFF 
GO
ALTER DATABASE [Ectobox] SET RECURSIVE_TRIGGERS OFF 
GO
ALTER DATABASE [Ectobox] SET  DISABLE_BROKER 
GO
ALTER DATABASE [Ectobox] SET AUTO_UPDATE_STATISTICS_ASYNC OFF 
GO
ALTER DATABASE [Ectobox] SET DATE_CORRELATION_OPTIMIZATION OFF 
GO
ALTER DATABASE [Ectobox] SET TRUSTWORTHY OFF 
GO
ALTER DATABASE [Ectobox] SET ALLOW_SNAPSHOT_ISOLATION OFF 
GO
ALTER DATABASE [Ectobox] SET PARAMETERIZATION SIMPLE 
GO
ALTER DATABASE [Ectobox] SET READ_COMMITTED_SNAPSHOT OFF 
GO
ALTER DATABASE [Ectobox] SET HONOR_BROKER_PRIORITY OFF 
GO
ALTER DATABASE [Ectobox] SET RECOVERY FULL 
GO
ALTER DATABASE [Ectobox] SET  MULTI_USER 
GO
ALTER DATABASE [Ectobox] SET PAGE_VERIFY CHECKSUM  
GO
ALTER DATABASE [Ectobox] SET DB_CHAINING OFF 
GO
ALTER DATABASE [Ectobox] SET FILESTREAM( NON_TRANSACTED_ACCESS = OFF ) 
GO
ALTER DATABASE [Ectobox] SET TARGET_RECOVERY_TIME = 60 SECONDS 
GO
ALTER DATABASE [Ectobox] SET DELAYED_DURABILITY = DISABLED 
GO
ALTER DATABASE [Ectobox] SET ACCELERATED_DATABASE_RECOVERY = OFF  
GO
EXEC sys.sp_db_vardecimal_storage_format N'Ectobox', N'ON'
GO
ALTER DATABASE [Ectobox] SET QUERY_STORE = OFF
GO
USE [Ectobox]
GO
/****** Object:  Table [dbo].[OPC_Station_Data]    Script Date: 11/20/2023 9:20:25 AM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[OPC_Station_Data](
	[ID] [int] IDENTITY(1,1) NOT NULL,
	[BatchID] [int] NOT NULL,
	[StationNum] [int] NULL,
	[BagNum] [int] NULL,
	[FillDateTime] [datetime2](7) NULL,
	[FillWeight] [float] NULL,
	[WeldUpperDateTime] [datetime2](7) NULL,
	[WeldUpperTemp] [float] NULL,
	[WeldLowerDateTime] [datetime2](7) NULL,
	[WeldLowerTemp] [float] NULL,
PRIMARY KEY CLUSTERED 
(
	[ID] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON, OPTIMIZE_FOR_SEQUENTIAL_KEY = OFF) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[OPC_Station_Data2]    Script Date: 11/20/2023 9:20:25 AM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[OPC_Station_Data2](
	[ID] [int] IDENTITY(1,1) NOT NULL,
	[BatchID] [int] NOT NULL,
	[StationNum] [int] NULL,
	[BagNum] [int] NULL,
	[FillDateTime] [datetime2](7) NULL,
	[FillWeight] [float] NULL,
	[WeldDateTimeStart] [datetime2](7) NULL,
	[WeldUpperTempStart] [float] NULL,
	[WeldLowerTempStart] [float] NULL,
	[WeldDateTimeEnd] [datetime2](7) NULL,
	[WeldUpperTempEnd] [float] NULL,
	[WeldLowerTempEnd] [float] NULL,
PRIMARY KEY CLUSTERED 
(
	[ID] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON, OPTIMIZE_FOR_SEQUENTIAL_KEY = OFF) ON [PRIMARY]
) ON [PRIMARY]
GO
/****** Object:  Table [dbo].[writetesting]    Script Date: 11/20/2023 9:20:25 AM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
CREATE TABLE [dbo].[writetesting](
	[_id] [int] IDENTITY(1,1) NOT NULL,
	[_name] [nvarchar](256) NULL,
	[_model] [nvarchar](256) NULL,
	[_timestamp] [bigint] NULL,
	[WeldUpperTemp] [float] NULL,
	[WeldLowerTemp] [float] NULL,
	[WeldJawClosed] [bit] NULL,
	[WeldJawOpen] [bit] NULL,
	[FillWeight] [float] NULL,
	[PinchRetract] [bit] NULL,
PRIMARY KEY CLUSTERED 
(
	[_id] ASC
)WITH (PAD_INDEX = OFF, STATISTICS_NORECOMPUTE = OFF, IGNORE_DUP_KEY = OFF, ALLOW_ROW_LOCKS = ON, ALLOW_PAGE_LOCKS = ON, OPTIMIZE_FOR_SEQUENTIAL_KEY = OFF) ON [PRIMARY]
) ON [PRIMARY]
GO
USE [master]
GO
ALTER DATABASE [Ectobox] SET  READ_WRITE 
GO
